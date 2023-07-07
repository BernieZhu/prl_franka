import tf
import rospy
import numpy as np
import time

import copy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
# from franka_interface import ArmInterface
from sensor_msgs.msg import JointState
from franka_gripper.msg import GraspAction, GraspGoal

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

from dvnets_franka_msgs.srv import GotoPose, GotoPoseResponse
from dvnets_franka_msgs.srv import SetGripper, SetGripperResponse
from dvnets_franka_msgs.srv import Home, HomeResponse

from control_msgs.msg import (
    GripperCommandAction, 
    GripperCommandFeedback,
    GripperCommandResult,
    GripperCommandGoal,
)

from octomap_msgs.msg import Octomap
class FrankaController:

    def __init__(self):
        # self.robot = ArmInterface()
        # --------- Modify as required ------------
        # Task-space controller parameters
        # stiffness gains
        self.P_pos = 50.
        self.P_ori = 25.
        # damping gains
        self.D_pos = 10.
        self.D_ori = 1.

        # home joint angles
        self.home_pose = [0.000,-0.785,0.0,-1.90,0.0,1.37,0.785]

        # home ee
        self.home_pose_ee = Pose()
        self.home_pose_ee.position.x =  0.256
        self.home_pose_ee.position.y =  0.000
        self.home_pose_ee.position.z =  0.776
        self.home_pose_ee.orientation.x = 0.916
        self.home_pose_ee.orientation.y = -0.379
        self.home_pose_ee.orientation.z = 0.117
        self.home_pose_ee.orientation.w = -0.048

        # self.forward_orientation = np.quaternion(0, 1, 0, -2.55195864440338e-12)
        self.gripper_rotation_offset = 0.5725969828761688 # radians

        # -----------------------------------------
        self.publish_rate = 100

        self.jacobian = None
        self.cartesian_pose = None
        self.cartesian_vel = None
        self.joint_states = None

        self.base_frame = 'panda_link0'
        self.tl = tf.TransformListener()

        self.joint_states_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self._on_joint_state,
            queue_size=1,
            tcp_nodelay=True
        )

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")

        rospy.sleep(2)

        box_pose = PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.position.x = 0.64
        box_pose.pose.position.y = -0.08
        box_pose.pose.position.z = -0.02 # slightly above the end effector
        box_pose.pose.orientation.w = 1.0
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=(1.0, 1.5, 0.1))


        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## We can get the name of the reference frame for this robot
        print ("============ Reference frame: %s" % self.arm_group.get_planning_frame())

        ## We can also print the name of the end-effector link for this group
        print ("============ Reference frame: %s" % self.arm_group.get_end_effector_link())

        ## We can get the name of the reference frame for this robot
        print ("============ Reference frame: %s" % self.hand_group.get_planning_frame())

        ## We can also print the name of the end-effector link for this group
        print ("============ Reference frame: %s" % self.hand_group.get_end_effector_link())
        ## We can get a list of all the groups in the robot
        print ("============ Robot Groups:")
        print (self.robot.get_group_names())

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot.
        print ("============ Printing robot state")
        print (self.robot.get_current_state())
        print ("============")

        self.gripper_action_client = actionlib.SimpleActionClient(
            '/franka_gripper/gripper_action',
            GripperCommandAction,
        )

        self.gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', 
            GraspAction,
        )
        self.gripper_grasp_client.wait_for_server()
        self.gripper_state = 'open'

        # goals
        self.goal_pos = None
        self.goal_ori = None

        self.pick_pose_pub = rospy.Publisher("/pick_pose", PoseStamped, queue_size=1)

        self.octomap_sub = rospy.Subscriber(
            '/octomap_binary',
            Octomap,
            self._on_octomap,
            queue_size=1,
            tcp_nodelay=True
        )

        self.octomap_added = False

    def _on_joint_state(self,msg):
        self.joint_states = msg

    def _quat_to_yaw(self, quat):
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]

    def send_to_neutral(self, req):
        rospy.wait_for_message('move_group/status', GoalStatusArray)
        commander = MoveGroupCommander('panda_arm')
        commander.set_named_target('ready')
        commander.go()
        return HomeResponse(True)

    def gripper_open(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.04
        goal.command.max_effort = 1000.0

        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result(rospy.Duration.from_sec(3.0))

    def gripper_close(self):
        goal = GraspGoal()
        goal.width = 0.0001
        goal.speed = 0.7
        goal.force = 10
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.07

        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result(rospy.Duration.from_sec(3.0))

    def set_gripper(self, req):
        open_gripper = req.gripper > 0.95
        if open_gripper:
            self.gripper_open()
        else:
            self.gripper_close()
        return SetGripperResponse(True)

    def goto(self, req):
        goal_pose = req.target_pose
        self.arm_group.set_pose_target(goal_pose.pose)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return GotoPoseResponse(True)
    
    def _on_octomap(self, msg):
        if not self.octomap_added:
            # Add the octomap as an obstacle to your scene
            # Name it 'octomap'
            self.scene.add_octomap(msg.data, 'octomap')
            self.octomap_added = True

if __name__ == "__main__":
    rospy.init_node("franka_controller_server")
    controller = FrankaController()

    # Let's wait for the first OctoMap message
    while not controller.octomap_added:
        rospy.sleep(0.1)

    goto_pose_service = rospy.Service('franka_goto_pose', GotoPose, controller.goto)
    set_gripper_service = rospy.Service('franka_set_gripper', SetGripper, controller.set_gripper)
    home_service = rospy.Service('franka_home', Home, controller.send_to_neutral)
    rospy.spin()