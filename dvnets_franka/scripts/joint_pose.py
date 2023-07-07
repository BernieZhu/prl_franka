import rospy
import moveit_commander

rospy.init_node("get_link_pose_node")

robot = moveit_commander.RobotCommander()

# Get the pose of 'panda_link'
link_id = 8
panda_link_pose = robot.get_link("panda_link{}".format(link_id)).pose()

print("Pose of panda_link{}:".format(link_id), panda_link_pose)

with open("poses/panda_link_pose.yaml", "w") as file:
    file.write(str(panda_link_pose))
    