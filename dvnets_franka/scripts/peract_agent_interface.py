import os
import sys
ARM_PATH = '/home/mshr/cliport_ndp/cliport_ndp'
sys.path.append(ARM_PATH)

import numpy as np
import pprint
import shutil
import pickle

import cv2
import rospy
import copy
import tf
import tf2_ros
import tf2_geometry_msgs
import threading
import pickle
from multiprocessing import Process, Manager
import torch

import hydra
from hydra import compose, initialize
from omegaconf import OmegaConf

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Joy, Image, CameraInfo, JointState
from visualization_msgs.msg import Marker

from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home
from rlbench.backend.observation import Observation
from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.demo import Demo
from pyrep.objects import VisionSensor
from rlbench.backend.utils import image_to_float_array, rgb_handles_to_mask

from arm import franka_c2farm_perceiver_lang_bc
from arm.clip.core.clip import tokenize
from arm import utils


class PeractAgentInterface:

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
			'step': 0,
			'prev_joy_states': None,
			'prev_pose': self.get_tf(self.base_frame, self.ee_frame),
			'new': False,
			'keypoint_done': False,
			'record': False,
		})

		# topics
		self.front_rgb_sub = rospy.Subscriber(self.cfg['topics']['front_rgb'], Image, self.front_rgb_cb)
		self.front_depth_sub = rospy.Subscriber(self.cfg['topics']['front_depth'], Image, self.front_depth_cb)
		self.front_camera_info_sub = rospy.Subscriber(self.cfg['topics']['front_camera_info'], CameraInfo, self.front_camera_info_cb)
		
		self.joy_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.joy_state_cb)
		self.joy_pose_sub = rospy.Subscriber(self.cfg['topics']['joy_pose'], PoseStamped, self.joy_pose_cb)

		self.joint_states_sub = rospy.Subscriber(self.cfg['topics']['joint_states'], JointState, self.joint_states_cb)
		self.target_pose_sub = rospy.Subscriber(self.cfg['topics']['target_pose'], Marker, self.target_pose_cb)

		self.pred_voxel_pub = rospy.Publisher(self.cfg['topics']['pred_voxel'], Image, latch=True, queue_size=10)

		# visualization
		self.peract_action_marker = self.get_ee_marker("package://franka_description/meshes/visual/hand.dae", [1.0, 0.0, 0.0, 0.6])
		self.peract_action_pub = rospy.Publisher('peract_action', Marker, queue_size=10)

		# controller
		rospy.wait_for_service('franka_goto_pose')
		self._franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
		rospy.wait_for_service('franka_set_gripper')
		self._franka_set_gripper = rospy.ServiceProxy('franka_set_gripper', SetGripper)

		# agent
		self.device = torch.device('cuda:0')
		self._load_agent()
		self.act_result = None

		# recorder
		self.frame_num = 0
		self.obs_frames = []

		# logs
		if self.cfg['logs']['save']:
			log_path = self.cfg['logs']['log_path']

			result_path = os.path.join(log_path,
									   self.cfg['logs']['task'],
									   f"episode{self.cfg['logs']['episode_id']}",
									   'act_result')

			if os.path.exists(result_path):
				resp = input(f"act_result for episode_id {self.cfg['logs']['episode_id']} exists. Overwrite? y/n: ")
				if resp == 'y':
					shutil.rmtree(result_path)
				else:
					sys.exit()					

		# language
		self.lang_goal = input("Language Goal: ")


	def _load_agent(self):
		# load config
		scene_bounds = self.cfg['agent']['scene_bounds']
		camera_resolution = self.cfg['agent']['camera_resolution']

		# cfg = OmegaConf.create(OmegaConf.to_yaml(conf, resolve=True))
		cfg_path = os.path.join(self.cfg['agent']['seed_path'], 'config.yaml')
		cfg = OmegaConf.load(cfg_path)
		# cfg['method']['num_latents'] = 256
		print(cfg)

		# load agent
		self.agent = franka_c2farm_perceiver_lang_bc.launch_utils.create_agent(
			cfg, None, depth_0bounds=scene_bounds, cam_resolution=camera_resolution)
		self.agent.build(training=False, device=self.device)

		# load pre-trained weights
		weights_path = os.path.join(self.cfg['agent']['seed_path'], 'weights',
									str(self.cfg['agent']['weight']))
		self.agent.load_weights(weights_path)

		print("Loaded: " + weights_path)

	'''
	Callbacks
	'''
	def front_rgb_cb(self, msg):
		self.curr_data['front_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")

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
	def get_ee_marker(self, mesh_resource, color):
		marker = Marker()

		marker = Marker()
		marker.header.frame_id = self.base_frame
		marker.header.stamp  = rospy.get_rostime()
		marker.ns = "robot"
		marker.id = 0
		marker.type = 10 # mesh
		marker.mesh_resource = mesh_resource
		marker.action = 0
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0

		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]
		marker.color.a = color[3]

		return marker

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
	def next_cond(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[2] < -0.8)) \
				and (joy.buttons[4] == 0)

	def execute_cond(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[2] < -0.8)) \
				and (joy.buttons[4] == 1)

	def new_lang_cond(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 0 and (joy.axes[2] > 0.8)) \
				and (joy.buttons[4] == 1)

	def save_frames(self):
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 0 and (joy.axes[1] > 0.8)) \
				and (joy.buttons[4] == 1)


	'''
	Main Funcs
	'''
	def get_obs(self, curr_data=None):
		obs = {}

		# option to pass in data
		if curr_data is None:
			curr_data = self.curr_data

		# front_camera
		front_camera_intrinsics = np.array(curr_data['front_camera_info'].K).reshape(3,3)
		obs['front_camera_intrinsics'] = torch.tensor([front_camera_intrinsics], device=self.device).unsqueeze(0)

		front_camera_extrinsics = self.pose_to_4x4mat(self.get_tf(self.base_frame, 'kinect_front_link').pose)
		obs['front_camera_extrinsics'] = torch.tensor([front_camera_extrinsics], device=self.device).unsqueeze(0)

		front_rgb = torch.tensor([copy.deepcopy(curr_data['front_rgb'])], device=self.device)
		front_rgb = front_rgb.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0)
		obs['front_rgb'] = front_rgb #torch.flip(front_rgb, dims=[2])

		front_depth = copy.deepcopy(curr_data['front_depth'])
		front_depth = image_to_float_array(front_depth, 1000.0)
		front_point_cloud = VisionSensor.pointcloud_from_depth_and_camera_params(
															front_depth,
															front_camera_extrinsics,
															front_camera_intrinsics)
		front_point_cloud = torch.tensor([front_point_cloud], device=self.device)
		front_point_cloud = front_point_cloud.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0)
		obs['front_point_cloud'] = front_point_cloud

		# collision
		obs['ignore_collisions'] = torch.tensor([[[1.0]]], device=self.device)

		# language
		lang_goal_tokens = tokenize([self.lang_goal])[0].numpy()
		lang_goal_tokens = torch.tensor([lang_goal_tokens], device=self.device).unsqueeze(0)
		obs['lang_goal_tokens'] = lang_goal_tokens

		# proprio
		finger_positions = np.array(curr_data['joint_states'].position)[-2:]
		gripper_open_amount = finger_positions[0] + finger_positions[1]
		gripper_open = (1.0 if (gripper_open_amount > 0.0385 + 0.0385) else 0.0)
		time = (1. - (self.state['step'] / float(self.cfg['agent']['episode_length'] - 1))) * 2. - 1.
		low_dim_state = torch.tensor([[[gripper_open, 
										finger_positions[0],
										finger_positions[1],
										time]]])
		obs['low_dim_state'] = low_dim_state

		# import pdb; pdb.set_trace()

		return obs

	def apply_gripper_offset(self, gripper_pose, offset=(0.0, 0.0, -0.107)):
		trans_matrix = tf.transformations.translation_matrix([offset[0], offset[1], offset[2]])
		rot_matrix = tf.transformations.quaternion_matrix([gripper_pose.orientation.x,
														   gripper_pose.orientation.y,
														   gripper_pose.orientation.z,
														   gripper_pose.orientation.w])
		rotated_offset = np.dot(rot_matrix, trans_matrix)

		gripper_trans_matrix = tf.transformations.translation_matrix([gripper_pose.position.x,
																	  gripper_pose.position.y,
																	  gripper_pose.position.z])

		rotated_and_translated_matrix = np.dot(gripper_trans_matrix, rotated_offset)

		final_quat = tf.transformations.quaternion_from_matrix(rotated_and_translated_matrix)
		final_trans = tf.transformations.translation_from_matrix(rotated_and_translated_matrix)

		final_pose = copy.deepcopy(gripper_pose)
		final_pose.position.x = final_trans[0]
		final_pose.position.y = final_trans[1]
		final_pose.position.z = final_trans[2]
		final_pose.orientation.x = final_quat[0]
		final_pose.orientation.y = final_quat[1]
		final_pose.orientation.z = final_quat[2]
		final_pose.orientation.w = final_quat[3]
		return final_pose

	def check_and_mkdirs(self, dir_path):
		if not os.path.exists(dir_path):
			os.makedirs(dir_path, exist_ok=True)

	def step(self):
		if self.cfg['logs']['save_all_frames']:
			self.obs_frames.append(copy.deepcopy(self.curr_data))
			self.frame_num += 1

		if self.save_frames():
			save_path = os.path.join(self.cfg['logs']['log_path'],
									 self.cfg['logs']['task'],
									 f"episode{self.cfg['logs']['episode_id']}",
									 self.cfg['logs']['save_all_folder'])

			self.check_and_mkdirs(save_path)


			for frame_num in range(len(self.obs_frames)):
				save_file = os.path.join(save_path,
										 f"{frame_num}.pkl")
				
				obs = self.get_obs(curr_data=self.obs_frames[frame_num])
				results = {
					'obs': obs,
				}
				with open(save_file, 'wb') as f:
					pickle.dump(results, f)

				print(f"Saved {frame_num} file: {save_file}")

			self.obs_frames = []

		elif self.new_lang_cond():
			self.lang_goal = input("Language Goal: ")
			self.state['step'] = int(input("Step: "))
	
		elif self.next_cond():
			observation = self.get_obs()
			self.act_result = self.agent.act(self.state['step'], observation,
										     deterministic=True)
			
			# visualize voxels
			voxel_grid = self.act_result.info['voxel_grid_depth0'].detach().cpu().numpy()[0]
			pred_q = self.act_result.info['q_depth0'].detach().cpu().numpy()[0]
			pred_trans_indices = self.act_result.info['voxel_idx_depth0'].detach().cpu().numpy()[0]
			# print(voxel_grid.shape, pred_q.shape, pred_trans_indices.shape)

			voxel_render = utils.visualise_voxel(voxel_grid, pred_q, pred_trans_indices, None,
												 rotation_amount=np.deg2rad(0), alpha=0.3)
			# print(voxel_render.shape)
			self.pred_voxel_pub.publish(self.cv_bridge.cv2_to_imgmsg(voxel_render, encoding='rgb8'))

			action = self.act_result.action

			# visualize action
			action_pose = PoseStamped()
			action_pose.header.frame_id = self.base_frame
			action_pose.pose.position.x = action[0]
			action_pose.pose.position.y = action[1]
			action_pose.pose.position.z = action[2] # + 0.107
			action_pose.pose.orientation.x = action[3]
			action_pose.pose.orientation.y = action[4]
			action_pose.pose.orientation.z = action[5]
			action_pose.pose.orientation.w = action[6]

			action_pose.pose = self.apply_gripper_offset(action_pose.pose, offset=(0.0, 0.0, -0.107))
			self.peract_action_marker.pose = copy.deepcopy(action_pose.pose)
			self.peract_action_pub.publish(self.peract_action_marker)

			gripper_open = action[7]
			print(f"Step: {self.state['step']} | Gripper Open: {action[7] > 0.99} | Ignore Collisions: {action[8] > 0.99}")


		elif self.execute_cond():
			if self.cfg['settings']['re_act_before_exec'] or self.act_result is None:
				observation = self.get_obs()
				self.act_result = self.agent.act(self.state['step'], observation,
											     deterministic=True)
				
				# visualize voxels
				voxel_grid = self.act_result.info['voxel_grid_depth0'].detach().cpu().numpy()[0]
				pred_q = self.act_result.info['q_depth0'].detach().cpu().numpy()[0]
				pred_trans_indices = self.act_result.info['voxel_idx_depth0'].detach().cpu().numpy()[0]
				# print(voxel_grid.shape, pred_q.shape, pred_trans_indices.shape)

				voxel_render = utils.visualise_voxel(voxel_grid, pred_q, pred_trans_indices, None,
													 rotation_amount=np.deg2rad(0), alpha=0.3)
				# print(voxel_render.shape)
				self.pred_voxel_pub.publish(self.cv_bridge.cv2_to_imgmsg(voxel_render, encoding='rgb8'))
				action = self.act_result.action
			else:
				action = self.act_result.action
			
			# execute
			action_pose = PoseStamped()
			action_pose.header.frame_id = self.base_frame
			action_pose.pose.position.x = action[0]
			action_pose.pose.position.y = action[1]
			action_pose.pose.position.z = action[2] # + 0.107
			action_pose.pose.orientation.x = action[3]
			action_pose.pose.orientation.y = action[4]
			action_pose.pose.orientation.z = action[5]
			action_pose.pose.orientation.w = action[6]

			action_pose.pose = self.apply_gripper_offset(action_pose.pose, offset=(0.0, 0.0, -0.107))
			self.goto_pose(action_pose)

			gripper_open = action[7]
			self._franka_set_gripper(gripper_open)

			# save act_result to disk
			if self.cfg['logs']['save']:
				log_path = self.cfg['logs']['log_path']
				self.check_and_mkdirs(log_path)

				result_path = os.path.join(log_path,
										   self.cfg['logs']['task'],
										   f"episode{self.cfg['logs']['episode_id']}",
										   'act_result')
				self.check_and_mkdirs(result_path)


				result_file = os.path.join(result_path,
										   f"{self.state['step']}.pkl")
				
				results = {
					'act_result': self.act_result,
					'lang_goal': self.lang_goal,
					'step': self.state['step'],
					'cfg': self.cfg,
				}
				with open(result_file, 'wb') as f:
					pickle.dump(results, f)

				print(f"Saved step {self.state['step']} act_result: {result_file}")


			self.state['step'] += 1


@hydra.main(config_path="../cfgs", config_name="peract_agent")
def main(cfg):
	# initialize(config_path="../cfgs", job_name="peract_demo")
	# cfg = compose(config_name="peract_demo")
	pprint.pprint(dict(cfg))


	rospy.init_node('peract_agent', anonymous=True)
	interface = PeractAgentInterface(cfg)

	while not rospy.is_shutdown():
		try:
			interface.step()
			interface.loop_rate.sleep()

		except KeyboardInterrupt:
			print("Shutting down demo interface.")

if __name__ == '__main__':
	main()
