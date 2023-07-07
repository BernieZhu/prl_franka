import rospy
import copy
import yaml

from geometry_msgs.msg import PoseStamped
from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home

# Initialize the node
rospy.init_node("pickup_object_node")

# Read object pose from file
def get_pose_from_file(file_path):
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
    object_pose = PoseStamped()
    object_pose.header.frame_id = "panda_link0"
    # object_pose.header.frame_id = "camera_color_optical_frame"
    object_pose.pose.position.x = data["pose"]["position"]["x"]
    object_pose.pose.position.y = data["pose"]["position"]["y"]
    object_pose.pose.position.z = data["pose"]["position"]["z"]
    object_pose.pose.orientation.x = data["pose"]["orientation"]["x"]
    object_pose.pose.orientation.y = data["pose"]["orientation"]["y"]
    object_pose.pose.orientation.z = data["pose"]["orientation"]["z"]
    object_pose.pose.orientation.w = data["pose"]["orientation"]["w"]
    return object_pose

# file_path = "poses/yellowcube.yaml"
# initial_pose = get_pose_from_file(file_path)
# object_pose = copy.deepcopy(initial_pose)

# Get the pose of the object from pose_topic
def pose_callback(msg):
    global initial_pose
    initial_pose = msg
initial_pose = PoseStamped()
rospy.Subscriber("pose_topic", PoseStamped, pose_callback)
# Wait until we receive the initial pose
while initial_pose.header.stamp == rospy.Time():
    rospy.sleep(0.1)
object_pose = copy.deepcopy(initial_pose)

def run_robot(object_pose):
    # Open the gripper
    rospy.wait_for_service('franka_set_gripper')
    set_gripper_service = rospy.ServiceProxy('franka_set_gripper', SetGripper)
    set_gripper_service(1)  # Open the gripper

    # Move the robot above the object
    object_pose.pose.position.z += 0.1  # Adjust to be above the object

    # Call the service to move the robot to the object's location
    rospy.wait_for_service('franka_goto_pose')
    goto_pose_service = rospy.ServiceProxy('franka_goto_pose', GotoPose)
    goto_pose_service(object_pose)

    # Lower the arm
    object_pose.pose.position.z -= 0.1
    rospy.wait_for_service('franka_goto_pose')
    goto_pose_service(object_pose)

    # Close the gripper to grasp the object
    rospy.wait_for_service('franka_set_gripper')
    set_gripper_service = rospy.ServiceProxy('franka_set_gripper', SetGripper)
    set_gripper_service(0)  # Close the gripper

    # Lift the object
    object_pose.pose.position.z += 0.1
    rospy.wait_for_service('franka_goto_pose')
    goto_pose_service(object_pose)

    # Move above the target and relase the object 
    file_path = "poses/plate.yaml"
    final_pose = get_pose_from_file(file_path)
    target_pose = copy.deepcopy(final_pose)
    rospy.wait_for_service('franka_goto_pose')
    goto_pose_service(target_pose)
    print("Finished!")

    rospy.wait_for_service('franka_set_gripper')
    set_gripper_service = rospy.ServiceProxy('franka_set_gripper', SetGripper)
    set_gripper_service(1)  # Open the gripper

    # Return to home position
    rospy.wait_for_service('franka_home')
    home_service = rospy.ServiceProxy('franka_home', Home)
    home_service()
    
# Wait for user input
while True:
    print(object_pose.pose)
    user_input = input("Press 'Y' to move the robot, 'N' to wait for the next pose, or 'S' to stop the node: ").upper()
    if user_input == 'Y':
        run_robot(object_pose)

    elif user_input == 'N':
        # Wait for the next pose
        while initial_pose.header.stamp == rospy.Time():
            rospy.sleep(0.1)
        object_pose = copy.deepcopy(initial_pose)
        continue

    elif user_input == 'S':
        # Stop the node
        print("Stopping the node...")
        rospy.signal_shutdown("User requested stop.")
        break

    else:
        print("Invalid input. Please type 'Y', 'N' or 'S'.")
        continue

