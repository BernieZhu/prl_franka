import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home

import copy

# Initialize the node
rospy.init_node("pickup_object_node")


# def goto_pose(ee_pose):
#     ee_cmd = copy.deepcopy(ee_pose)

#     # weird Franka 45 deg shift
#     offset_45 = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(45.0))
#     target_ee_quat = [ee_cmd.pose.orientation.x,
#                         ee_cmd.pose.orientation.y,
#                         ee_cmd.pose.orientation.z,
#                         ee_cmd.pose.orientation.w]
#     rotated_target_ee_quat = tf.transformations.quaternion_multiply(target_ee_quat, offset_45)
    
#     # norm = np.linalg.norm(np.array(rotated_target_ee_quat), ord=2)
#     ee_cmd.pose.orientation.x = rotated_target_ee_quat[0]
#     ee_cmd.pose.orientation.y = rotated_target_ee_quat[1]
#     ee_cmd.pose.orientation.z = rotated_target_ee_quat[2]
#     ee_cmd.pose.orientation.w = rotated_target_ee_quat[3]

#     # self.controller.goto(ee_cmd)
#     rospy.wait_for_service('franka_goto_pose')
#     franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
#     succces = franka_goto(ee_cmd)
#     print("Move to pose:\n", ee_cmd.pose)

rospy.wait_for_service('franka_set_gripper')
set_gripper_service = rospy.ServiceProxy('franka_set_gripper', SetGripper)
set_gripper_service(1)  # Open the gripper

# Read object position from file
with open("poses/box.txt", "r") as file:
    lines = file.readlines()

object_pose = PoseStamped()
object_pose.header.frame_id = "panda_link0"
object_pose.pose.position.x = float(lines[1].split(':')[1])
object_pose.pose.position.y = float(lines[2].split(':')[1])
object_pose.pose.position.z = float(lines[3].split(':')[1])
object_pose.pose.orientation.x = float(lines[5].split(':')[1])
object_pose.pose.orientation.y = float(lines[6].split(':')[1])
object_pose.pose.orientation.z = float(lines[7].split(':')[1])
object_pose.pose.orientation.w = float(lines[8].split(':')[1])
# goto_pose(object_pose)

pose_above_object = copy.deepcopy(object_pose)
rospy.wait_for_service('franka_goto_pose')
goto_pose_service = rospy.ServiceProxy('franka_goto_pose', GotoPose)
succcess = goto_pose_service(pose_above_object)
print("Move to pose:\n", object_pose.pose)

