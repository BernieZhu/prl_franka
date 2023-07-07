#!/usr/bin/env python

# Python libs
import sys

# numpy and scipy
import numpy as np
from time import time
import pickle
import os

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
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from dvnets_franka.srv import BBoxSegmentation, BBoxSegmentationRequest, BBoxSegmentationResponse
from franka_moveit_controller import FrankaController

# misc
import hydra


class DemoInterface:

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

        # rgb
        self.bridge = CvBridge()
        self.rgb = None
        self.pre_rgb = None
        # self.image_sub = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)

        # depth
        self.depth = None
        self.pre_depth = None
        # self.depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)

        # pointcloud
        self.pc = None
        # self.pc_sub = message_filters.Subscriber("/kinect2/qhd/points", PointCloud2)

        # camera info
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

        # setup dataset
        self.setup_data()

    def setup_data(self):
        data_path = self.cfg['data_path']
        episode_id = self.cfg['episode_id']

        if not os.path.isdir(data_path):
            os.makedirs(data_path)

        color_dir = os.path.join(data_path, 'color')
        if not os.path.isdir(color_dir):
            os.makedirs(color_dir)

        depth_dir = os.path.join(data_path, 'depth')
        if not os.path.isdir(depth_dir):
            os.makedirs(depth_dir)

        action_dir = os.path.join(data_path, 'action')
        if not os.path.isdir(action_dir):
            os.makedirs(action_dir)

        reward_dir = os.path.join(data_path, 'reward')
        if not os.path.isdir(reward_dir):
            os.makedirs(reward_dir)

        info_dir = os.path.join(data_path, 'info')
        if not os.path.isdir(info_dir):
            os.makedirs(info_dir)

        color_pkl = os.path.join(color_dir, '{}.pkl'.format(episode_id))
        depth_pkl = os.path.join(depth_dir, '{}.pkl'.format(episode_id))
        action_pkl = os.path.join(action_dir, '{}.pkl'.format(episode_id))
        reward_pkl = os.path.join(reward_dir, '{}.pkl'.format(episode_id))
        info_pkl = os.path.join(info_dir, '{}.pkl'.format(episode_id))

        if os.path.isfile(color_pkl) and os.path.isfile(depth_pkl) and os.path.isfile(action_pkl) and os.path.isfile(reward_pkl) and os.path.isfile(info_pkl):

            with open(color_pkl, 'rb') as f:
                self.data['color'] = self.data['color'] + pickle.load(f)

            with open(depth_pkl, 'rb') as f:
                self.data['depth'] = self.data['depth'] + pickle.load(f)

            with open(action_pkl, 'rb') as f:
                self.data['action'] = self.data['action'] + pickle.load(f)

            with open(reward_pkl, 'rb') as f:
                self.data['reward'] = self.data['reward'] + pickle.load(f)

            with open(info_pkl, 'rb') as f:
                self.data['info'] = self.data['info'] + pickle.load(f)

            print("Loading existing {}.pkl | Total Steps: {}".format(episode_id, len(self.data['action'])))

    def nothing(self, x):
        pass

    def crop_callback(self, x):
        print(x)

    def mouse_callback(self, event, x, y, flags, params):
       # t = time()
        if event == cv2.EVENT_LBUTTONDOWN:
          self.mouse_start = (x, y)
          self.mouse_processed = True
          if self.verbose:
              print('Start Mouse Position: '+str(x)+', '+str(y))

        elif event == cv2.EVENT_LBUTTONUP:
           self.mouse_end = (x, y)
           self.mouse_processed = False
           if self.verbose:
               print ('End Mouse Position: '+str(x)+', '+str(y))
           self.rotation_processed = False

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
        display = np.array(display, dtype=np.uint8)

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
        # cv2.destroyAllWindows()

        # action function
        start_action = self.sweep_start if self.is_sweep_task else self.pick_object
        end_action = self.sweep_end if self.is_sweep_task else self.place_object

        # keyboard shortcuts
        if key == 27: # ESC
            self.clear()
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
        elif key == ord('s'):
            self.save_step()
        elif key == ord('u'):
            self.undo_save_step()
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

    def extract_centroid(self):
        cx = self.cfg['image']['crop_x'] 
        cy = self.cfg['image']['crop_y']
        scale = self.cfg['image']['scale']

        # bbox
        self.bbox_x, self.bbox_y = self.mouse_start[0], self.mouse_start[1]
        self.bbox_width, self.bbox_height = (self.mouse_end[0] - self.mouse_start[0]), (self.mouse_end[1] - self.mouse_start[1])
        min_height = self.cfg['pointcloud']['min_height_offset'] + (cv2.getTrackbarPos('Segment Min Height','Interface') - self.cfg['pointcloud']['min_height_ticks']/2) / self.cfg['pointcloud']['min_height_scale']
        max_depth = self.cfg['segmentor']['max_depth']

        if self.verbose:
            print(self.bbox_x, self.bbox_y, self.bbox_width, self.bbox_height, min_height)

        # segment-out cluster
        try:
            req = BBoxSegmentationRequest()

            req.x = float(self.bbox_x * scale + cx)
            req.y = float(self.bbox_y * scale + cy)
            req.width = float(self.bbox_width * scale)
            req.height = float(self.bbox_height * scale)
            req.min_height = float(min_height)
            req.max_depth = float(max_depth)

            resp = self.segmentor_srv(req)
        except rospy.ServiceException, e:
            print "Segmentation Service Failed: %s"%e
            return

        cluster = resp.cluster
        centroid = resp.centroid

        self.centroid_stamped = PoseStamped()
        self.centroid_stamped.header = cluster.header
        # self.centroid_stamped.header.frame_id = 'rs200_camera'
        self.centroid_stamped.pose.position.x = centroid.x
        self.centroid_stamped.pose.position.y = centroid.y
        self.centroid_stamped.pose.position.z = centroid.z

        self.centroid_pub.publish(self.centroid_stamped)
        self.segmented_pc.publish(cluster)

    def select_rotation(self):
        if self.bbox_x:
            mid_x, mid_y = self.bbox_x + (self.bbox_width / 2.0), self.bbox_y + (self.bbox_height / 2.0)
        else:
            mid_x, mid_y = self.mouse_end[0] + (self.mouse_end[0] - self.mouse_start[0]) / 2.0, self.mouse_end[1] + (self.mouse_end[1] - self.mouse_start[1]) / 2.0

        end_x, end_y = self.mouse_end[0], self.mouse_end[1]
        width, height = (end_x - mid_x), (end_y - mid_y)
        theta_rad = np.arctan2(width, height)
        theta_deg = np.rad2deg(theta_rad)

        n_rotations = self.cfg[self.task_cfg]['n_rotations']
        self.theta_deg_discrete = np.round(theta_deg / n_rotations) * float(n_rotations) # 36 degree discretization
        self.theta_rad_discrete = np.deg2rad(self.theta_deg_discrete)

        self.grasp_right = (mid_x + self.grasp_len/2.0 * np.sin(self.theta_rad_discrete), mid_y + self.grasp_len/2.0 * np.cos(self.theta_rad_discrete))
        self.grasp_left  = (mid_x - self.grasp_len/2.0 * np.sin(self.theta_rad_discrete), mid_y - self.grasp_len/2.0 * np.cos(self.theta_rad_discrete))

        # publish rotation
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta_rad_discrete + np.deg2rad(45.0) + np.deg2rad(180))
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0.7071068, 0, 0.7071068])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0, -0.7071068, 0.7071068])
        quat = tf.transformations.quaternion_multiply(quat, [1, 0, 0, 0])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 1, 0, 0])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 0, 1, 0])
        # quat = tf.transformations.quaternion_multiply(quat, [0.3817607, 0.9242614, 0.0, 0.0])
        # quat = tf.transformations.quaternion_multiply(quat, [0, 1, 0, 0])

        self.centroid_stamped.pose.orientation.x = quat[0]
        self.centroid_stamped.pose.orientation.y = quat[1]
        self.centroid_stamped.pose.orientation.z = quat[2]
        self.centroid_stamped.pose.orientation.w = quat[3]
        self.centroid_pub.publish(self.centroid_stamped)

        if self.verbose:
            print (self.grasp_right, self.grasp_left)

    def pick_object(self):
        base_goal = self.tl.transformPose(self.base_frame, self.centroid_stamped)

        pre_grasp_pose = copy.deepcopy(base_goal)
        pre_grasp_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        pre_grasp_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        pre_grasp_pose.pose.position.z += self.cfg[self.task_cfg]['pre_grasp_height_offset']
        print(pre_grasp_pose.pose)

        self.grasp_pose = copy.deepcopy(base_goal)
        self.grasp_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        self.grasp_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        self.grasp_pose.pose.position.z += self.cfg[self.task_cfg]['grasp_height_offset']
        self.grasp_pose.pose.position.z = np.clip(self.grasp_pose.pose.position.z, self.cfg[self.task_cfg]['lowest_height'], self.cfg[self.task_cfg]['highest_height'])

        if not self.pre_grasp:
            self.controller.send_to_neutral()
            self.controller.goto(pre_grasp_pose)
            self.controller.gripper_open()

            with self.lock:
                self.pre_rgb = np.copy(self.rgb[:,:,::-1])
                self.pre_depth = np.copy(self.depth)

            self.grasp_pix = (self.bbox_x + (self.bbox_width / 2.0), self.bbox_y + (self.bbox_height / 2.0), self.theta_rad_discrete)

            self.pre_grasp = True
            self.post_grasp = False
        else:
            self.controller.goto(self.grasp_pose)
            self.pre_grasp = False
            self.controller.gripper_close()
            self.controller.goto(pre_grasp_pose)
            if self.cfg[self.task_cfg]['home_after_pick']:
                self.controller.send_to_neutral()
            self.post_grasp = True

    def place_object(self):
        base_goal = self.tl.transformPose(self.base_frame, self.centroid_stamped)

        pre_place_pose = copy.deepcopy(base_goal)
        pre_place_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        pre_place_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        pre_place_pose.pose.position.z += self.cfg[self.task_cfg]['pre_place_height_offset']

        self.place_pose = copy.deepcopy(base_goal)
        self.place_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        self.place_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        self.place_pose.pose.position.z += self.cfg[self.task_cfg]['place_height_offset']
        self.place_pose.pose.position.z = np.clip(self.place_pose.pose.position.z, self.cfg[self.task_cfg]['lowest_height'], self.cfg[self.task_cfg]['highest_height'])
        self.place_pix = (self.bbox_x + (self.bbox_width / 2.0), self.bbox_y + (self.bbox_height / 2.0), self.theta_rad_discrete)

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
        base_goal = self.tl.transformPose(self.base_frame, self.centroid_stamped)

        pre_grasp_pose = copy.deepcopy(base_goal)
        pre_grasp_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        pre_grasp_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        pre_grasp_pose.pose.position.z = self.cfg[self.task_cfg]['pre_sweep_offset']
        print(pre_grasp_pose.pose)

        self.grasp_pose = copy.deepcopy(base_goal)
        self.grasp_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        self.grasp_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        self.grasp_pose.pose.position.z = self.cfg[self.task_cfg]['sweep_height']

        self.controller.send_to_neutral()
        self.controller.gripper_close()

        with self.lock:
            self.pre_rgb = np.copy(self.rgb[:,:,::-1])
            self.pre_depth = np.copy(self.depth)

        self.grasp_pix = (self.bbox_x + (self.bbox_width / 2.0), self.bbox_y + (self.bbox_height / 2.0), self.theta_rad_discrete)

        self.controller.goto(self.grasp_pose)
        self.pre_grasp = False
        self.post_grasp = True

    def sweep_end(self):
        base_goal = self.tl.transformPose(self.base_frame, self.centroid_stamped)

        pre_place_pose = copy.deepcopy(base_goal)
        pre_place_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        pre_place_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        pre_place_pose.pose.position.z = self.cfg[self.task_cfg]['pre_sweep_offset']

        self.place_pose = copy.deepcopy(base_goal)
        self.place_pose.pose.position.x += self.cfg[self.task_cfg]['grasp_x_offset']
        self.place_pose.pose.position.y += self.cfg[self.task_cfg]['grasp_y_offset']
        self.place_pose.pose.position.z = self.cfg[self.task_cfg]['sweep_height']
        self.place_pix = (self.bbox_x + (self.bbox_width / 2.0), self.bbox_y + (self.bbox_height / 2.0), self.theta_rad_discrete)

        self.controller.goto(self.place_pose)
        self.pre_place = False
        self.controller.send_to_neutral()
        self.post_grasp = False

    def save_step(self):
        # color
        color = self.pre_rgb
        self.data['color'].append([color])

        print(color.shape)
        print(np.max(color), np.min(color), np.mean(color))

        # depth
        depth = np.array(self.pre_depth, dtype=np.float32) / 1000.0
        self.data['depth'].append([depth])

        print(depth.shape)
        print(np.max(depth), np.min(depth), np.mean(depth))

        # action
        action = {
            'pose0': (np.array([self.grasp_pose.pose.position.x,
                                self.grasp_pose.pose.position.y,
                                self.grasp_pose.pose.position.z]),
                      np.array([self.grasp_pose.pose.orientation.x,
                                self.grasp_pose.pose.orientation.y,
                                self.grasp_pose.pose.orientation.z,
                                self.grasp_pose.pose.orientation.w])),
            'pose1': (np.array([self.place_pose.pose.position.x,
                                self.place_pose.pose.position.y,
                                self.place_pose.pose.position.z]),
                      np.array([self.place_pose.pose.orientation.x,
                                self.place_pose.pose.orientation.y,
                                self.place_pose.pose.orientation.z,
                                self.place_pose.pose.orientation.w])),

            'pix0': self.grasp_pix,
            'pix1': self.place_pix,
        }
        self.data['action'].append(action)
        print(self.grasp_pix, self.place_pix)

        # info
        info = {}
        lang_goal = raw_input("Lang: ")
        if lang_goal == "\p" and len(self.data['info']) > 0:
            info['lang_goal'] = self.data['info'][-1]['lang_goal']
        else:
            info['lang_goal'] = lang_goal

        transform = self.tf2_buffer.lookup_transform(self.base_frame,
                                                    'kinect2_ir_optical_frame', #source frame
                                                    rospy.Time(0), #get the tf at first available time
                                                    rospy.Duration(1.0)) #wait for 1 second
        # camera info
        info['camera_info'] = {
            'height': self.camera_info.height,
            'width': self.camera_info.width,
            'D': self.camera_info.D,
            'K': self.camera_info.K,
            'R': self.camera_info.R,
            'P': self.camera_info.P,
            'frame_id': self.camera_info.header.frame_id,
            'position': (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
            'rotation': (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        }
        print(info['camera_info'])

        self.data['info'].append(info)

        reward = eval(raw_input("Reward: "))
        self.data['reward'].append(reward)

        self.save_pkl_files()

    def save_pkl_files(self):
        # save to pickle files
        data_path = self.cfg['data_path']
        episode_id = self.cfg['episode_id']

        color_pkl = os.path.join(data_path, "color", "{}.pkl".format(episode_id))
        depth_pkl = os.path.join(data_path, "depth", "{}.pkl".format(episode_id))
        action_pkl = os.path.join(data_path, "action", "{}.pkl".format(episode_id))
        info_pkl = os.path.join(data_path, "info", "{}.pkl".format(episode_id))
        reward_pkl = os.path.join(data_path, "reward", "{}.pkl".format(episode_id))

        if len(self.data['color']) > 0:
            with open(color_pkl, 'wb') as f:
                pickle.dump(self.data['color'], f)

            with open(depth_pkl, 'wb') as f:
                pickle.dump(self.data['depth'], f)

            with open(action_pkl, 'wb') as f:
                pickle.dump(self.data['action'], f)

            with open(info_pkl, 'wb') as f:
                pickle.dump(self.data['info'], f)

            with open(reward_pkl, 'wb') as f:
                pickle.dump(self.data['reward'], f)

            print(self.data)
            print("Saved: {}.pkl | Total Steps: {}".format(episode_id, len(self.data['action'])))
        else:
            if os.path.exists(color_pkl):
                os.remove(color_pkl)

            if os.path.exists(depth_pkl):
                os.remove(depth_pkl)

            if os.path.exists(action_pkl):
                os.remove(action_pkl)

            if os.path.exists(info_pkl):
                os.remove(info_pkl)

            if os.path.exists(reward_pkl):
                os.remove(reward_pkl)

            print("Removed: {}.pkl".format(episode_id))

    def undo_save_step(self):
        confirm = raw_input("Undo Step - Confirm (Y/n): ")
        if confirm == "Y":
            self.data['color'] = self.data['color'][:-1]
            self.data['depth'] = self.data['depth'][:-1]
            self.data['action'] = self.data['action'][:-1]
            self.data['info'] = self.data['info'][:-1]
            self.data['reward'] = self.data['reward'][:-1]

            print("Undone - Total Steps: {}".format(len(self.data['color'])))
            self.save_pkl_files()

    def done_step(self):
        if len(self.data['color']) > 0:
            self.controller.send_to_neutral()

            with self.lock:
                self.pre_rgb = np.copy(self.rgb[:,:,::-1])
                self.pre_depth = np.copy(self.depth)

            # color
            self.data['color'].append([self.pre_rgb])

            # depth
            depth = np.array(self.pre_depth, dtype=np.float32) / 1000.0
            self.data['depth'].append([depth])

            # action
            action = None
            self.data['action'].append(action)
            print(self.grasp_pix, self.place_pix)

            # info
            info = {}
            lang_goal = raw_input("Lang: ")
            info['lang_goal'] = lang_goal
            self.data['info'].append(info)

            reward = eval(raw_input("Reward: "))
            self.data['reward'].append(reward)

            self.save_pkl_files()

            exit = raw_input("Exit (Y/n): ")
            if exit == 'Y':
                print("Exiting.")
                quit()
        else:
            print("No steps to save!")

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


@hydra.main(config_path="../cfgs/demo.yaml")
def main(cfg):
    '''Initializes and cleanup ros node'''
    rospy.init_node('demo_interface', anonymous=True)

    while not rospy.is_shutdown():
        try:
            interface = DemoInterface(cfg)
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()