#!/usr/bin/env python

# Python libs
import sys

# numpy and scipy
import numpy as np
from time import time
import pickle
import os
import json

# OpenCV
import cv2

# Ros libraries
import tf
import copy
import roslib
import rospy
import threading
import message_filters

import tf2_ros
import tf2_geometry_msgs

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from dvnets_franka.srv import BBoxSegmentation, BBoxSegmentationRequest, BBoxSegmentationResponse
from franka_moveit_controller import FrankaController

from demo_interface import DemoInterface
from dvnets_franka_msgs.srv import Transporter, TransporterRequest, TransporterResponse

# misc
import hydra


class AgentInterface(DemoInterface):

    def __init__(self, cfg):
        '''Initialize ros publisher, ros image_sub'''
        # config
        self.cfg = cfg
        self.verbose = self.cfg['verbose']
        self.task = self.cfg['task']
        self.task_cfg = self.task if self.task in self.cfg else 'manipulation'
        print("Task Config: {}".format(self.task_cfg))

        # special cases
        self.is_sweep_task = self.cfg[self.task_cfg]['sweeping_action']
        if self.is_sweep_task:
            print("Sweep Controller.")
            
        # controller
        self.controller = FrankaController(self.cfg)

        # agent
        self.agent_srv = rospy.ServiceProxy("/dvnets", Transporter)
        self.pick_pub = rospy.Publisher("/pick_pose", PoseStamped, queue_size=1)
        self.place_pub = rospy.Publisher("/place_pose", PoseStamped, queue_size=1)

        # rgb
        self.bridge = CvBridge()
        self.rgb = None
        self.rgb_raw = None
        self.pre_rgb = None
        # self.image_sub = message_filters.Subscriber("/r200/camera/color/image_raw", Image)

        # depth
        self.depth = None
        self.depth_raw = None
        self.pre_depth = None
        # self.depth_sub = message_filters.Subscriber("/r200/camera/depth/image_raw", Image)

        # pointcloud
        self.pc = None
        # self.pc_sub = message_filters.Subscriber("/r200/camera/depth_registered/points", PointCloud2)

        self.lang_goal = ''

        self.camera_info = None

        # time sync
        # self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.pc_sub], 10, 1.0)
        # self.ts.registerCallback(self.sensor_callback)

        # segmentor
        self.segment_processed = True
        self.rotation_processed = True
        self.segmentor_srv = rospy.ServiceProxy("/bbox_segmentor", BBoxSegmentation)
        self.centroid_pub = rospy.Publisher("/bbox_centroid", PoseStamped, queue_size=1)
        self.segmented_pc = rospy.Publisher("/bbox_segmented_pc", PointCloud2, queue_size=1)
        self.centroid_stamped = PoseStamped()

        # mouse event
        self.mouse_start = (None, None)
        self.mouse_end = (None, None)
        self.mouse_processed = True

        # bbox
        self.bbox_x = None
        self.bbox_y = None
        self.bbox_width = None
        self.bbox_height = None

        # grasp
        self.theta_deg_discrete = 0.0
        self.theta_rad_discrete = 0.0

        self.grasp_len = 65
        self.grasp_right = (None, None)
        self.grasp_left  = (None, None)
        self.grasp_pose = None
        self.grasp_pix = (None, None)
        self.pre_grasp = False
        self.post_grasp = False
        self.pre_place = False
        self.place_pose = None
        self.place_pix = (None, None)

        # listener
        self.base_frame = 'panda_link0'
        self.tl = tf.TransformListener()

        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # lock
        self.lock = threading.Lock()

        # data
        self.data = {
            'color': [],
            'depth': [],
            'action': [],
            'reward': [],
            'info': [],
        }

        # create displays
        cv2.namedWindow('Interface', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Interface', 320*2, 160*2)
        cv2.resizeWindow('Interface', 960, 540)

        cv2.setMouseCallback('Interface', self.mouse_callback)
        cv2.createTrackbar('Segment Min Height', 'Interface',
                           self.cfg['pointcloud']['min_height'],
                           self.cfg['pointcloud']['min_height_ticks'],
                           self.nothing)


        if self.verbose :
            print("subscribed to /kinect2/qhd/image_color_rect")
            print("subscribed to /kinect2/qhd/image_depth_rect")


    def nothing(self, x):
        pass

    def crop_callback(self, x):
        print(x)

    def sensor_callback(self, rgb, depth, pc, camera_info):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        try:
            # ros_data.encoding = "uc16"
            self.rgb = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            self.depth = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            self.pc = pc.data
            self.camera_info = camera_info
        except Exception as e:
            print(e)

        # crop RGB and Depth
        cx = self.cfg['image']['crop_x']
        cy = self.cfg['image']['crop_y']
        scale = self.cfg['image']['scale']
        w = int(self.cfg['image']['width'] * scale)
        h = int(self.cfg['image']['height'] * scale)

        self.rgb = self.rgb[cy:cy+h, cx:cx+w, :]
        self.depth = self.depth[cy:cy+h, cx:cx+w]

        self.rgb = cv2.resize(self.rgb, (320, 160))
        self.depth = cv2.resize(self.depth, (320, 160))

        # draw selection box
        display = self.rgb.copy()
        if not self.mouse_processed and self.bbox_x is not None:
            cv2.rectangle(display, (self.bbox_x, self.bbox_y), (self.bbox_x + self.bbox_width, self.bbox_y + self.bbox_height), color=(0, 255, 0), thickness=2)

            grasp_left = (int(self.grasp_left[0]), int(self.grasp_left[1]))
            grasp_right = (int(self.grasp_right[0]), int(self.grasp_right[1]))
            cv2.line(display, grasp_left, grasp_right, color=(0, 200, 0), thickness=2)
            cv2.circle(display, grasp_left, radius=5, color=(0, 200, 0), thickness=2)
            cv2.circle(display, grasp_right, radius=5, color=(0, 200, 0), thickness=2)

        # show image
        cv2.imshow('Interface', display)
        key = cv2.waitKey(2)

        # action function
        start_action = self.sweep_start if self.is_sweep_task else self.pick_object
        end_action = self.sweep_end if self.is_sweep_task else self.place_object

        # keyboard shortcuts
        if key == 27: # ESC
            self.clear()
        elif key == ord('n'):
            self.get_next_action()
        elif key == ord('b'):
            self.segment_processed = False
        elif key == ord('p'):
            self.pre_grasp = False
            start_action()
        elif key == ord('l'):
            if self.post_grasp:
                end_action()
        elif key == ord('5'): # Return
            if self.pre_grasp:
                start_action()
            elif self.post_grasp:
                end_action()
        elif key == ord('0'):
            self.controller.send_to_neutral()
        elif key == ord('['):
            self.controller.gripper_open()
        elif key == ord(']'):
            self.controller.gripper_close()
        elif key == ord('{'):
            rospy.sleep(10)
            self.controller.gripper_open()
        elif key == ord('}'):
            rospy.sleep(10)
            self.controller.gripper_close()
        elif key == ord('d'):
            self.done_step()

        # segment bbox
        if not self.segment_processed:
            self.extract_centroid()
            self.segment_processed = True
            self.rotation_processed = False

        # select rotation
        if not self.rotation_processed:
            self.select_rotation()
            self.rotation_processed = True

    def get_quat_from_theta(self, theta):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -theta - np.deg2rad(45.0))
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0.7071068, 0, 0.70710689])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0, -0.7071068, 0.7071068])
        quat = tf.transformations.quaternion_multiply(quat, [1, 0, 0, 0])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 1, 0, 0])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0, 1, 0])

        q = Quaternion()
        q.x = quat[0]
        q.y = quat[1]
        q.z = quat[2]
        q.w = quat[3]
        return q

    def extract_patch_centroid(self, x, y):
        cx = self.cfg['image']['crop_x']
        cy = self.cfg['image']['crop_y']
        scale = self.cfg['image']['scale']

        # bbox
        patch_size = self.cfg[self.task_cfg]['patch_size']
        min_height = self.cfg['pointcloud']['min_height_offset'] + (cv2.getTrackbarPos('Segment Min Height','Interface') - self.cfg['pointcloud']['min_height_ticks']/2) / self.cfg['pointcloud']['min_height_scale']
        max_depth = self.cfg['segmentor']['max_depth']

        # segment-out cluster
        try:
            req = BBoxSegmentationRequest()

            req.x = float(x - patch_size//2)
            req.y = float(y - patch_size//2)
            req.width = float(patch_size)
            req.height = float(patch_size)
            req.min_height = float(min_height)
            req.max_depth = float(max_depth)

            resp = self.segmentor_srv(req)
        except rospy.ServiceException, e:
            print "Segmentation Service Failed: %s"%e
            return

        cluster = resp.cluster
        centroid = resp.centroid
        return centroid

    def get_next_action(self):
        cx = self.cfg['image']['crop_x']
        cy = self.cfg['image']['crop_y']
        scale = self.cfg['image']['scale']

        lang_goal = raw_input("Lang: ")

        if lang_goal == '\p' and self.lang_goal:
            lang_goal = self.lang_goal
            print("Prev. Goal: {}".format(lang_goal))
        else:
            self.lang_goal = lang_goal

        transform = self.tf2_buffer.lookup_transform(self.base_frame,
                                                    'kinect2_ir_optical_frame', #source frame
                                                    rospy.Time(0), #get the tf at first available time
                                                    rospy.Duration(1.0)) #wait for 1 second
        position = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        rotation = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

        try:
            req = TransporterRequest()

            req.colors = [self.bridge.cv2_to_imgmsg(np.copy(self.rgb[:,:,::-1]), encoding="passthrough")]
            req.depths = [self.bridge.cv2_to_imgmsg(np.copy(self.depth), encoding='passthrough')]
            req.lang_goal = lang_goal

            resp = self.agent_srv(req)
        except rospy.ServiceException, e:
            print "Segmentation Service Failed: %s"%e
            return

        self.grasp_pose = PoseStamped()
        self.grasp_pose.header.stamp = rospy.Time.now()
        self.grasp_pose.header.frame_id = self.base_frame

        # custom rotation for sweep
        if self.is_sweep_task:
            start_xy = np.array(resp.pick_xyt[:2])
            final_xy = np.array(resp.place_xyt[:2])

            theta = np.arctan2(final_xy[1]-start_xy[1], final_xy[0]-start_xy[0])
            theta += np.pi

            resp.pick_xyt = (resp.pick_xyt[0], resp.pick_xyt[1], theta)
            resp.place_xyt = (resp.place_xyt[0], resp.place_xyt[1], theta)

        pick_xy = resp.pick_xyt[0] * scale + cx, resp.pick_xyt[1] * scale + cy
        # self.grasp_pose.pose = resp.pick_pose
        grasp_centroid = self.extract_patch_centroid(*pick_xy)
        self.grasp_pose.pose.position.x = grasp_centroid.x + self.cfg[self.task_cfg]['grasp_x_offset']
        self.grasp_pose.pose.position.y = grasp_centroid.y + self.cfg[self.task_cfg]['grasp_y_offset']
        self.grasp_pose.pose.position.z = grasp_centroid.z + self.cfg[self.task_cfg]['grasp_height_offset']
        self.grasp_pose.pose.position.z = np.clip(self.grasp_pose.pose.position.z, self.cfg[self.task_cfg]['lowest_height'], self.cfg[self.task_cfg]['highest_height'])
        self.grasp_pose.pose.orientation = self.get_quat_from_theta(resp.pick_xyt[2])
        self.pick_pub.publish(self.grasp_pose)

        self.place_pose = PoseStamped()
        self.place_pose.header.stamp = rospy.Time.now()
        self.place_pose.header.frame_id = self.base_frame

        place_xy = resp.place_xyt[0] * scale + cx, resp.place_xyt[1] * scale + cy
        # self.place_pose.pose = resp.place_pose
        place_centroid = self.extract_patch_centroid(*place_xy)
        self.place_pose.pose.position.x = place_centroid.x + self.cfg[self.task_cfg]['grasp_x_offset']
        self.place_pose.pose.position.y = place_centroid.y + self.cfg[self.task_cfg]['grasp_y_offset']
        self.place_pose.pose.position.z = place_centroid.z + self.cfg[self.task_cfg]['place_height_offset']
        self.place_pose.pose.position.z = np.clip(self.place_pose.pose.position.z, self.cfg[self.task_cfg]['lowest_height'], self.cfg[self.task_cfg]['highest_height'])
        self.place_pose.pose.orientation = self.get_quat_from_theta(resp.place_xyt[2])
        self.place_pub.publish(self.place_pose)

    def pick_object(self):
        pre_grasp_pose = copy.deepcopy(self.grasp_pose)
        pre_grasp_pose.pose.position.z += self.cfg[self.task_cfg]['pre_grasp_height_offset']

        if not self.pre_grasp:
            self.controller.send_to_neutral()
            self.controller.goto(pre_grasp_pose)
            self.controller.gripper_open()

            self.pre_rgb = np.copy(self.rgb[:,:,::-1])
            self.pre_depth = np.copy(self.depth)

            self.pre_grasp = True
            self.post_grasp = False
        else:
            self.controller.goto(self.grasp_pose)
            self.pre_grasp = False
            self.controller.gripper_close()
            self.controller.goto(pre_grasp_pose)
            # if self.cfg[self.task_cfg]['home_after_pick']:
            #     self.controller.send_to_neutral()
            self.post_grasp = True

    def place_object(self):

        pre_place_pose = copy.deepcopy(self.place_pose)
        pre_place_pose.pose.position.z += self.cfg[self.task_cfg]['pre_place_height_offset']
        pre_place_pose.pose.position.z = np.clip(pre_place_pose.pose.position.z, self.cfg[self.task_cfg]['lowest_height'], self.cfg[self.task_cfg]['highest_height'])

        if not self.pre_place:
            # self.controller.send_to_neutral()
            self.controller.goto(pre_place_pose)
            self.pre_place = True
        else:
            self.controller.goto(self.place_pose)
            self.pre_place = False
            self.controller.gripper_open()
            self.controller.goto(pre_place_pose)
            self.controller.send_to_neutral()
            self.post_grasp = False

    def sweep_start(self):
        grasp_pose = copy.deepcopy(self.grasp_pose)
        grasp_pose.pose.position.z = self.cfg[self.task_cfg]['sweep_height']

        self.controller.send_to_neutral()
        self.controller.gripper_close()

        self.controller.goto(grasp_pose)
        self.pre_grasp = False
        self.post_grasp = True

    def sweep_end(self):
        place_pose = copy.deepcopy(self.place_pose)
        place_pose.pose.position.z = self.cfg[self.task_cfg]['sweep_height']

        self.controller.goto(place_pose)
        self.pre_place = False
        self.controller.send_to_neutral()
        self.post_grasp = False

    def done_step(self):
        done = raw_input("Done? Y/n: ")

        if done == 'Y':
            reward = eval(raw_input("Reward: "))
            with open(self.cfg['results_json'], 'w') as f:
                json.dump(
                    {'mean_reward': reward}, f
                )

    def clear(self):
        self.mouse_processed = True
        self.segment_processed = True
        self.rotation_processed = True
        self.pre_grasp = False
        self.post_grasp = False
        self.pre_place = False

    def reset(self):
        self.grasp_pose = None
        self.place_pose = None
        self.grasp_pix = None
        self.place_pix = None
        self.pre_rgb = None
        self.pre_depth = None


@hydra.main(config_path="../cfgs/agent.yaml")
def main(cfg):
    '''Initializes and cleanup ros node'''
    rospy.init_node('agent_interface', anonymous=True)
    agent_interface = AgentInterface(cfg)
    
    while not rospy.is_shutdown():
        try:
            # rospy.spin()
            rgb = rospy.wait_for_message('/kinect2/qhd/image_color_rect', Image)
            depth = rospy.wait_for_message('/kinect2/qhd/image_depth_rect', Image)
            pc = rospy.wait_for_message('/kinect2/qhd/points', PointCloud2)
            camera_info = rospy.wait_for_message('/kinect2/qhd/camera_info', CameraInfo)

            agent_interface.sensor_callback(rgb, depth, pc, camera_info)

        except KeyboardInterrupt:
            print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()