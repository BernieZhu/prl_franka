import rospy
import yaml

from geometry_msgs.msg import PoseStamped

def load_pose_from_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    
    pose = PoseStamped()
    # pose.header.frame_id = data["pose"]['frame_id']  # replace with the key containing frame_id in your yaml file
    pose.pose.position.x = data["pose"]['position']['x']  # replace with the keys containing position in your yaml file
    pose.pose.position.y = data["pose"]['position']['y']
    pose.pose.position.z = data["pose"]['position']['z']
    pose.pose.orientation.x = data["pose"]['orientation']['x']  # replace with the keys containing orientation in your yaml file
    pose.pose.orientation.y = data["pose"]['orientation']['y']
    pose.pose.orientation.z = data["pose"]['orientation']['z']
    pose.pose.orientation.w = data["pose"]['orientation']['w']

    return pose

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("pose_publisher_node")

    # Load the pose from yaml file
    file_path = "poses/yellowcube.yaml"
    pose = load_pose_from_yaml(file_path)

    # Create a publisher
    pose_topic = "pose_topic"
    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)

    # Allow some time for the publisher to connect to subscribers
    rospy.sleep(1.0)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        print(f"Published pose to {pose_topic}")
        rate.sleep()
