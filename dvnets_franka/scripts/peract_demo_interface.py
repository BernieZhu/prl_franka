import os
import sys
import numpy as np
import pprint
import shutil

import cv2
import rospy
import copy
import tf
import tf2_ros
import tf2_geometry_msgs
import threading
import pickle
from multiprocessing import Process, Manager

import hydra
from hydra import compose, initialize
from omegaconf import OmegaConf

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, Image, CameraInfo, JointState
from visualization_msgs.msg import Marker

from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home
from rlbench.backend.observation import Observation
from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.demo import Demo


class PeractDemoInterface:

	def __init__(self, cfg):
		self.cfg = cfg

		# setup
		self.loop_rate = rospy.Rate(cfg['ros']['loop_rate'])
		self.base_frame = 'panda_link0'
		self.ee_frame = 'panda_K'

		# tools
		self.cv_bridge = CvBridge()
		self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
		self.mp = Manager()

		# data
		self.curr_data = self.mp.dict({
			'front_rgb': None,
			'front_depth': None,
			'front_camera_info': None,

			'joy_states': None,
			'joy_pose': None,

			'joint_states': None,
			'target_pose': None,
			'gripper_pose': None,
		})

		# states
		self.state = self.mp.dict({
			'prev_joy_states': None,
			'prev_pose': self.get_tf(self.base_frame, self.ee_frame),
			'new': False,
			'keypoint_done': False,
			'record': False,
		})

		# keypoint data
		self.keypoint_data = self.mp.list()
		self.keypoint_idxs = self.mp.list([0])

		# topics
		self.front_rgb_sub = rospy.Subscriber(self.cfg['topics']['front_rgb'], Image, self.front_rgb_cb)
		self.front_depth_sub = rospy.Subscriber(self.cfg['topics']['front_depth'], Image, self.front_depth_cb)
		self.front_camera_info_sub = rospy.Subscriber(self.cfg['topics']['front_camera_info'], CameraInfo, self.front_camera_info_cb)
		
		self.joy_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.joy_state_cb)
		self.joy_pose_sub = rospy.Subscriber(self.cfg['topics']['joy_pose'], PoseStamped, self.joy_pose_cb)

		self.joint_states_sub = rospy.Subscriber(self.cfg['topics']['joint_states'], JointState, self.joint_states_cb)
		self.target_pose_sub = rospy.Subscriber(self.cfg['topics']['target_pose'], Marker, self.target_pose_cb)

		# controller
		rospy.wait_for_service('franka_goto_pose')
		self._franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
		rospy.wait_for_service('franka_set_gripper')
		self._franka_set_gripper = rospy.ServiceProxy('franka_set_gripper', SetGripper)

		# language
		self.lang_goal = input("Language Goal: ")

	'''
	Callbacks
	'''
	def front_rgb_cb(self, msg):
		self.curr_data['front_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

	def front_depth_cb(self, msg):
		self.curr_data['front_depth'] = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")

	def front_camera_info_cb(self, msg):
		self.curr_data['front_camera_info'] = msg

	def joy_state_cb(self, msg):
		self.state['prev_joy_states'] = self.curr_data['joy_states']
		self.curr_data['joy_states'] = msg

	def joy_pose_cb(self, msg):
		self.curr_data['joy_pose'] = msg
		self.curr_data['gripper_pose'] = self.get_tf(self.base_frame, self.ee_frame)

	def joint_states_cb(self, msg):
		self.curr_data['joint_states'] = msg

	def target_pose_cb(self, msg):
		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = self.base_frame
		pose_stamped.pose = msg.pose

		self.curr_data['target_pose'] = pose_stamped
		self.state['new'] = True

	'''
	Helper Funcs
	'''
	def get_tf(self, target_frame, source_frame):
		transform = self.tf2_buffer.lookup_transform(target_frame,
													 source_frame,
													 rospy.Time(0), #get the tf at first available time
													 rospy.Duration(1.0)) #wait for 1 second
		
		pose_stamped = PoseStamped()
		pose_stamped.header = transform.header
		pose_stamped.pose.position.x = transform.transform.translation.x
		pose_stamped.pose.position.y = transform.transform.translation.y
		pose_stamped.pose.position.z = transform.transform.translation.z

		pose_stamped.pose.orientation.x = transform.transform.rotation.x
		pose_stamped.pose.orientation.y = transform.transform.rotation.y
		pose_stamped.pose.orientation.z = transform.transform.rotation.z
		pose_stamped.pose.orientation.w = transform.transform.rotation.w

		return pose_stamped

	def pose_to_4x4mat(self, pose):
		basetrans = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
		baserot = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
		return np.matmul(basetrans, baserot)

	def goto_pose(self, ee_pose):
		ee_cmd = copy.deepcopy(ee_pose)

		# weird Franka 45 deg shift
		offset_45 = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(45.0))
		target_ee_quat = [ee_cmd.pose.orientation.x,
						  ee_cmd.pose.orientation.y,
						  ee_cmd.pose.orientation.z,
						  ee_cmd.pose.orientation.w]
		rotated_target_ee_quat = tf.transformations.quaternion_multiply(target_ee_quat, offset_45)
		
		# norm = np.linalg.norm(np.array(rotated_target_ee_quat), ord=2)
		ee_cmd.pose.orientation.x = rotated_target_ee_quat[0]
		ee_cmd.pose.orientation.y = rotated_target_ee_quat[1]
		ee_cmd.pose.orientation.z = rotated_target_ee_quat[2]
		ee_cmd.pose.orientation.w = rotated_target_ee_quat[3]

		# self.controller.goto(ee_cmd)
		succces = self._franka_goto(ee_cmd)


	'''
	Joystick Funcs
	'''
	def record_pose_cond(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[3] == 1 and joy.buttons[4] == 1)

	def record_grasp_cond(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[1] > 0.8 or joy.axes[1] < -0.8)) \
				and prev_joy.buttons[4] == 1

	def goto_last_keypoint_pose(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[2] < -0.8)) \
				and (joy.buttons[4] == 1)

	'''
	Main Funcs
	'''
	def record_goto(self):
		start_idx = len(self.keypoint_data)
		print("Recording now ...")
		while self.state['record']:
			self.keypoint_data.append(copy.deepcopy(self.curr_data))
			self.loop_rate.sleep()
		end_idx = len(self.keypoint_data) - 1
		self.keypoint_idxs.append(end_idx)
		print(f"Recorded {end_idx - start_idx + 1} frames.")


	def record_grasp(self):
		start_idx = len(self.keypoint_data)
		print("Recording now ...")
		self.keypoint_data.append(copy.deepcopy(self.curr_data))
		end_idx = len(self.keypoint_data) - 1
		self.keypoint_idxs[-1] = end_idx
		print(f"Recorded 1 frame.")


	def get_obs(self, frame, misc):
		finger_positions = np.array(frame['joint_states'].position)[-2:]
		gripper_open_amount = finger_positions[0] + finger_positions[1]

		gripper_pose = np.array([
			frame['gripper_pose'].pose.position.x,
			frame['gripper_pose'].pose.position.y,
			frame['gripper_pose'].pose.position.z,
			frame['gripper_pose'].pose.orientation.x,
			frame['gripper_pose'].pose.orientation.y,
			frame['gripper_pose'].pose.orientation.z,
			frame['gripper_pose'].pose.orientation.w,
		])

		obs = Observation(
			left_shoulder_rgb=None,
			left_shoulder_depth=None,
			left_shoulder_point_cloud=None,
			right_shoulder_rgb=None,
			right_shoulder_depth=None,
			right_shoulder_point_cloud=None,
			overhead_rgb=None,
			overhead_depth=None,
			overhead_point_cloud=None,
			wrist_rgb=None,
			wrist_depth=None,
			wrist_point_cloud=None,
			front_rgb=None,
			front_depth=None,
			front_point_cloud=None,
			left_shoulder_mask=None,
			right_shoulder_mask=None,
			overhead_mask=None,
			wrist_mask=None,
			front_mask=None,
			joint_velocities=np.array(frame['joint_states'].velocity)[:7],
			joint_positions=np.array(frame['joint_states'].position)[:7],
			joint_forces=np.array(frame['joint_states'].effort)[:7],
			gripper_open=(1.0 if (gripper_open_amount > 0.0385 + 0.0385) else 0.0),
			gripper_pose=gripper_pose,
			gripper_matrix=self.pose_to_4x4mat(frame['gripper_pose'].pose),
			gripper_touch_forces=None,
			gripper_joint_positions=finger_positions,
			task_low_dim_state=None,
			ignore_collisions=True, # TODO: fix
			misc=misc,
		)
		return obs


	def save_keypoint(self):
		# make directories
		def check_and_mkdirs(dir_path):
			if not os.path.exists(dir_path):
				os.makedirs(dir_path, exist_ok=True)

		save_path = os.path.join(self.cfg['demo']['save_path'], self.cfg['demo']['task'])
		episode_idx = self.cfg['demo']['episode']
		variation_idx = self.cfg['demo']['variation']

		episode_path = os.path.join(save_path, 'all_variations', 'episodes', f"episode{episode_idx}")
		check_and_mkdirs(episode_path)

		front_rgb_path = os.path.join(episode_path, 'front_rgb')
		check_and_mkdirs(front_rgb_path)
		front_depth_path = os.path.join(episode_path, 'front_depth')
		check_and_mkdirs(front_depth_path)

		# misc (camera_info etc)
		misc =dict()
		frame0 = self.keypoint_data[0]
		misc['front_camera_intrinsics'] = np.array(frame0['front_camera_info'].K).reshape(3,3)
		misc['front_camera_extrinsics'] = self.pose_to_4x4mat(self.get_tf(self.base_frame, 'kinect_front_link').pose)
		misc['front_camera_near'] = 0.5
		misc['front_camera_far'] = 4.5

		misc['keypoint_idxs'] = np.array(list(self.keypoint_idxs))[1:]

		observations = []
		for f_idx, frame in enumerate(self.keypoint_data):
			save_idx = f_idx

			front_rgb = frame['front_rgb']
			front_depth = frame['front_depth']

			front_rgb_filename = os.path.join(front_rgb_path, f'{f_idx}.png')
			cv2.imwrite(front_rgb_filename, front_rgb)

			front_depth_filename = os.path.join(front_depth_path, f'{f_idx}.png')
			cv2.imwrite(front_depth_filename, front_depth)

			observations.append(self.get_obs(frame, misc))

		demo = Demo(observations, random_seed=self.cfg['demo']['random_seed'])
		demo.variation_number = variation_idx

		low_dim_obs_path = os.path.join(episode_path, 'low_dim_obs.pkl')
		with open(low_dim_obs_path, 'wb') as f:
			pickle.dump(demo, f)

		variation_number_path = os.path.join(episode_path, 'variation_number.pkl')
		with open(variation_number_path, 'wb') as f:
			pickle.dump(variation_idx, f)

		descriptions = self.lang_goal.split(",")
		descriptions_path = os.path.join(episode_path, 'variation_descriptions.pkl')
		with open(descriptions_path, 'wb') as f:
			pickle.dump(descriptions, f)

		print(f"Saved {len(self.keypoint_data)} frames to {save_path}")


	def undo_keypoint(self):
		# import pdb; pdb.set_trace()
		if len(self.keypoint_data) > 0:
			self.keypoint_data = self.mp.list(self.keypoint_data[:self.keypoint_idxs[-2] + 1])
			self.keypoint_idxs = self.mp.list(self.keypoint_idxs[:-1])


	def step(self):
	
		# start only if new target pose was received
		if self.state['new'] and self.record_pose_cond():
			
			# go to last recorded pose
			prev_pose = self.state['prev_pose']
			if self.cfg['settings']['replay_from_prev_pose']:
				self.goto_pose(prev_pose)

			self.state['record'] = True
			self.state['keypoint_done'] = False

			rec_process = Process(target=self.record_goto, args=())
			rec_process.start()
			target_pose = self.curr_data['target_pose']
			self.goto_pose(target_pose)

			self.state['record'] = False
			self.state['keypoint_done'] = True
			rec_process.join()

			self.state['new'] = False

			# ask to save
			if self.cfg['settings']['ask_to_save']:
				resp = input('Save keypoint trajectory? (y/n)\n')
				if resp == 'y':
					self.save_keypoint()
					self.state['prev_pose'] = target_pose
				else:
					self.undo_keypoint()
			else:
				self.save_keypoint()
				self.state['prev_pose'] = target_pose

		# undo pose
		if self.goto_last_keypoint_pose():
			prev_pose = self.state['prev_pose']
			self.goto_pose(prev_pose)

		# record grasp change
		if self.record_grasp_cond():

			self.state['record'] = True
			self.state['keypoint_done'] = False

			ax = self.curr_data['joy_states'].axes[1]
			gripper_state = 1.0 if ax > 0.8 else 0.0
			self._franka_set_gripper(gripper_state)
			self.record_grasp()

			if self.cfg['settings']['ask_to_save']:
				resp = input('Save keypoint trajectory? (y/n)\n')
				if resp == 'y':
					self.save_keypoint()
				else:
					self.undo_keypoint()
			else:
				self.save_keypoint()


@hydra.main(config_path="../cfgs", config_name="peract_demo")
def main(cfg):
	# initialize(config_path="../cfgs", job_name="peract_demo")
	# cfg = compose(config_name="peract_demo")
	pprint.pprint(dict(cfg))

	save_path = os.path.join(cfg['demo']['save_path'], cfg['demo']['task'])
	episode_idx = cfg['demo']['episode']
	variation_idx = cfg['demo']['variation']
	episode_path = os.path.join(save_path, 'all_variations', 'episodes', f"episode{episode_idx}")
	if os.path.exists(episode_path):
		resp = input(f"{cfg['demo']['task']} - Episode {episode_idx} already exists. Overwrite? (y/n)\n")
		if resp == 'y':
			shutil.rmtree(episode_path)
		else:
			sys.exit()

	rospy.init_node('peract_demo', anonymous=True)
	interface = PeractDemoInterface(cfg)

	while not rospy.is_shutdown():
		try:
			interface.step()
			interface.loop_rate.sleep()

		except KeyboardInterrupt:
			print("Shutting down demo interface.")

if __name__ == '__main__':
	main()
