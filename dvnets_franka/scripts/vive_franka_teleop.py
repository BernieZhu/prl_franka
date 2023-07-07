import tf
import copy
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker

from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home

import hydra
from hydra import compose, initialize
from omegaconf import OmegaConf


class ViveFrankaTeleop:

	def __init__(self, cfg):
		self.cfg = cfg
		# self.controller = FrankaController(self.cfg)
		rospy.wait_for_service('franka_goto_pose')
		self._franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
		rospy.wait_for_service('franka_set_gripper')
		self._franka_set_gripper = rospy.ServiceProxy('franka_set_gripper', SetGripper)
		rospy.wait_for_service('franka_home')
		self._franka_home = rospy.ServiceProxy('franka_home', Home)
		print("Found all services")

		self.base_frame = 'panda_link0'
		self.ee_frame = 'panda_K'
		self.tl = tf.TransformListener()
		self.tb = tf.TransformBroadcaster()

		self.gripper_state = 'close'
		self.last_joy = None
		
		self.vive_pose = PoseStamped()
		self.vive_origin_pose = PoseStamped()
		self.vive_origin_pose.pose.orientation.w = 1.0

		self.ee_pose = PoseStamped()
		self.ee_pose_origin = PoseStamped()
		self.ee_pose_origin.pose.orientation.w = 1.0

		self.ee_marker = self.get_ee_marker("package://franka_description/meshes/visual/hand.dae", [0.0, 1.0, 0.0, 0.6])
		self.ee_marker_pub = rospy.Publisher('vive_gripper', Marker, queue_size=10)

		self.target_ee_marker = self.get_ee_marker("package://franka_description/meshes/visual/hand.dae", [1.0, 1.0, 0.0, 0.6])
		self.target_ee_marker_pub = rospy.Publisher('target_gripper', Marker, queue_size=10)
		self.target_pose_45 = PoseStamped()
		self.target_pose_45.pose.orientation.w = 1.0
		print("Initialized")

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

	def get_ee_pose(self):
		if self.tl.frameExists(self.base_frame) and self.tl.frameExists(self.ee_frame):
			t = self.tl.getLatestCommonTime(self.base_frame, self.ee_frame)
			position, quaternion = self.tl.lookupTransform(self.base_frame, self.ee_frame, t)
			return position, quaternion
		raise Exception("tf not avaialble between {} and {}".format(self.base_frame, self.ee_frame))

	def apply_offset(self, pose):
		pose.position.x += self.cfg['manipulation']['offsets']['trans_x']
		pose.position.y += self.cfg['manipulation']['offsets']['trans_y']
		pose.position.z += self.cfg['manipulation']['offsets']['trans_z']

		pose_quat = [pose.orientation.x,
					 pose.orientation.y,
					 pose.orientation.z,
					 pose.orientation.w]

		offset_quat = tf.transformations.quaternion_from_euler(
			np.deg2rad(self.cfg['manipulation']['offsets']['rot_x']),
			np.deg2rad(self.cfg['manipulation']['offsets']['rot_y']),
			np.deg2rad(self.cfg['manipulation']['offsets']['rot_z']))

		rotated = tf.transformations.quaternion_multiply(offset_quat, pose_quat)
		norm = np.linalg.norm(np.array(rotated), ord=2)
		pose.orientation.x = rotated[2] # / norm
		pose.orientation.y = rotated[1] # / norm
		pose.orientation.z = -rotated[0] # / norm
		pose.orientation.w = rotated[3] # / norm

		return pose

	def save_ee_pose(self):
		ee_pos, ee_quat = self.get_ee_pose()

		self.ee_pose.pose.position.x = ee_pos[0]
		self.ee_pose.pose.position.y = ee_pos[1]
		self.ee_pose.pose.position.z = ee_pos[2]

		self.ee_pose.pose.orientation.x = ee_quat[0]
		self.ee_pose.pose.orientation.y = ee_quat[1]
		self.ee_pose.pose.orientation.z = ee_quat[2]
		self.ee_pose.pose.orientation.w = ee_quat[3]

	def quat_diff(self, quat1, quat2):
		"""
		returns qr, where q2 = qr * q1
		"""
		q1_inv = [0.0, 0.0, 0.0, 1.0]
		q1_inv[0] = quat1.x
		q1_inv[1] = quat1.y
		q1_inv[2] = quat1.z
		q1_inv[3] = -quat1.w # Negate for inverse

		q2 = [0.0, 0.0, 0.0, 1.0]
		q2[0] = quat2.x
		q2[1] = quat2.y
		q2[2] = quat2.z
		q2[3] = quat2.w

		qr = tf.transformations.quaternion_multiply(q2, q1_inv)		
		return qr

	def callback(self, joy, pose):
		# display
		self.ee_marker.pose = self.apply_offset(pose.pose)
		
		# rot_offset = self.quat_diff(self.vive_origin_pose.pose.orientation, self.ee_marker.pose.orientation)
		# ee_origin_quat = [self.ee_pose_origin.pose.orientation.x, 
		# 				  self.ee_pose_origin.pose.orientation.y, 
		# 				  self.ee_pose_origin.pose.orientation.z, 
		# 				  self.ee_pose_origin.pose.orientation.w]
		# quat_offset = tf.transformations.quaternion_multiply(rot_offset, ee_origin_quat)
		# norm = np.linalg.norm(np.array(quat_offset), ord=2)
		# self.ee_marker.pose.orientation.x = quat_offset[0] / norm
		# self.ee_marker.pose.orientation.y = quat_offset[1] / norm
		# self.ee_marker.pose.orientation.z = quat_offset[2] / norm
		# self.ee_marker.pose.orientation.w = quat_offset[3] / norm

		self.ee_marker_pub.publish(self.ee_marker)
		self.tb.sendTransform((self.ee_marker.pose.position.x, self.ee_marker.pose.position.y, self.ee_marker.pose.position.z),
							  (self.ee_marker.pose.orientation.x, self.ee_marker.pose.orientation.y, self.ee_marker.pose.orientation.z, self.ee_marker.pose.orientation.w),
							  rospy.Time.now(),
							  "marker", self.base_frame)

		# home
		if (joy.buttons[2] == 1 and joy.axes[2] > 0.8):
			print("Going home")
			self._franka_home()

		# open
		elif (joy.buttons[2] == 1 and joy.axes[1] > 0.8) and self.last_joy.buttons[4] == 0:
			print("Gripper Open")
			self._franka_set_gripper(1.0)
			self.gripper_state = 'open'

		# close
		elif (joy.buttons[2] == 1 and joy.axes[1] < -0.8):
			print("Gripper Close")
			self._franka_set_gripper(0.0)
			self.gripper_state = 'close'

		# record pose
		if (joy.buttons[0] == 1 and self.last_joy.buttons[0] == 0) and \
			(joy.buttons[4] == 0):
			print("Set Origin Pose")
			self.target_ee_marker.pose = self.ee_marker.pose
			self.target_ee_marker_pub.publish(self.target_ee_marker)

			self.tb.sendTransform((self.target_ee_marker.pose.position.x, 
								   self.target_ee_marker.pose.position.y, 
								   self.target_ee_marker.pose.position.z),
								  (self.target_ee_marker.pose.orientation.x, 
								   self.target_ee_marker.pose.orientation.y, 
								   self.target_ee_marker.pose.orientation.z, 
								   self.target_ee_marker.pose.orientation.w),
								  rospy.Time.now(),
								  "target", self.base_frame)

			time = rospy.Time.now()
			self.tb.sendTransform((0.0, 0.0, 0.103),
								  (0.000, 0.000, 0.000, 1.0),
								  time,
								  "target_K", 'target')

			self.tb.sendTransform((0.0, 0.0, 0.103),
								  (0.000, 0.000, 0.383, 0.924),
								  time,
								  "target_K45", 'target')

			self.tb.sendTransform((0.0, 0.029, 0.058),
								  (0.000, 0.000, 0.000, 1.0),
								  time,
								  "target_left", 'target')

			self.tb.sendTransform((-0.0, -0.029, 0.058),
								  (0.000, 0.000, 0.000, 1.0),
								  time,
								  "target_right", 'target')

			# self.tl.waitForTransform(self.base_frame, "target_K45", rospy.Time(), rospy.Duration(4.0))
			# while not rospy.is_shutdown():
			# 	try:
			# 		now = rospy.Time.now()
			# 		self.tl.waitForTransform(self.base_frame, "target_K45", now, rospy.Duration(4.0))
			# 		position, quaternion = self.tl.lookupTransform(self.base_frame, "target_K45")

			# 		self.target_pose_45.pose.position.x = position[0]
			# 		self.target_pose_45.pose.position.y = position[1]
			# 		self.target_pose_45.pose.position.z = position[2]

			# 		self.target_pose_45.pose.orientation.x = quaternion[0]
			# 		self.target_pose_45.pose.orientation.y = quaternion[1]
			# 		self.target_pose_45.pose.orientation.z = quaternion[2]
			# 		self.target_pose_45.pose.orientation.w = quaternion[3]

			# 		break
			# 	except Exception as e:
			# 		print(e)
			# 		pass


		# reset orientation
		# if (joy.buttons[4] == 1 and joy.axes[2] < -0.90):
		# 	print("Saved EE and Vive pose")
		# 	ee_pos, ee_quat = self.get_ee_pose()
		# 	self.ee_pose_origin.pose.position.x = ee_pos[0]
		# 	self.ee_pose_origin.pose.position.y = ee_pos[1]
		# 	self.ee_pose_origin.pose.position.z = ee_pos[2]
		# 	self.ee_pose_origin.pose.orientation.x = ee_quat[0]
		# 	self.ee_pose_origin.pose.orientation.y = ee_quat[1]
		# 	self.ee_pose_origin.pose.orientation.z = ee_quat[2]
		# 	self.ee_pose_origin.pose.orientation.w = ee_quat[3]

		# 	self.vive_origin_pose = pose

		# goto recorded pose
		elif (joy.buttons[3] == 1 and joy.buttons[4] == 0) and \
		   (self.last_joy.buttons[3] == 0 and self.last_joy.buttons[4] == 0):
			print("Go to pose")
			self.save_ee_pose()

			# translation		
			# ee_cmd = copy.deepcopy(self.target_pose_45)
			# ee_cmd.header.frame_id = self.base_frame
			# marker.header.stamp  = rospy.get_rostime()

			ee_cmd = copy.deepcopy(self.ee_pose)
			ee_cmd.pose = self.target_ee_marker.pose

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


		# publish TFs

		self.last_joy = joy


# @hydra.main(config_path="../cfgs/teleop.yaml")

def main():
	initialize(config_path="../cfgs", job_name="teleop")
	cfg = compose(config_name="teleop")
	print(cfg)

	rospy.init_node('vive_franka_teleop', anonymous=True)
	teleop = ViveFrankaTeleop(cfg)

	while not rospy.is_shutdown():
		try:
			joy = rospy.wait_for_message('/vive_right', Joy)
			pose = rospy.wait_for_message('/right_controller_as_posestamped', PoseStamped)
			teleop.callback(joy, pose)

		except KeyboardInterrupt:
			print("Shutting down vive_franka_teleop")

if __name__ == '__main__':
	main()