#!/usr/bin/env python
import rospy
import yaml

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    if len(sys.argv) < 1:
        rospy.logerr('Missing yaml file in args')
        sys.exit(0)
    else:
        yaml_file = sys.argv[1]
        with open(yaml_file, 'r') as file:
            calib = yaml.safe_load(file)

        print(calib)

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()

        while not rospy.is_shutdown():
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "panda_link0"
            static_transformStamped.child_frame_id = "kinect_front_link"

            static_transformStamped.transform.translation.x = calib['transformation']['x'] + 3.3231198872119637e-02
            static_transformStamped.transform.translation.y = calib['transformation']['y'] - 5.2495543167471091e-02
            static_transformStamped.transform.translation.z = calib['transformation']['z'] - 9.6718257146388847e-03

            static_transformStamped.transform.rotation.x = calib['transformation']['qx']
            static_transformStamped.transform.rotation.y = calib['transformation']['qy']
            static_transformStamped.transform.rotation.z = calib['transformation']['qz']
            static_transformStamped.transform.rotation.w = calib['transformation']['qw']

            broadcaster.sendTransform(static_transformStamped)
            rospy.sleep(0.01)