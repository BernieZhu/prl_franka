#! /usr/bin/env python
import rospy
from dvnets_franka.srv import BBoxSegmentation, BBoxSegmentationResponse
import copy
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
import tf
import sensor_msgs.point_cloud2 as pcl2

import numpy as np


DEBUG_PC_PUB = False
REFERENCE_FRAME = 'panda_link0'

class BBoxSegmentor:

    def __init__(self, pc_topic_name, bbox_seg_srv_name):
        self._pc_sub = rospy.Subscriber(pc_topic_name, PointCloud2, self.pc_sub)
        self._seg_pub = rospy.Publisher('bbox_segmentor/segmented_pc_debug', PointCloud2, queue_size=2, latch=True)
        self._centroid_pub = rospy.Publisher('bbox_segmentor/segmented_centroid', PointStamped, queue_size=2, latch=True)
        self._bbox_srv = rospy.Service(bbox_seg_srv_name, BBoxSegmentation, self.segment)

        self._tl = tf.TransformListener()
        print("Started BBox Segmentation Service")

    def pc_sub(self, msg):
        self._raw_pc = msg

    def clean_nan_inf(self, val):
        if np.isnan(val) or np.isinf(val):
            return 0.0
        else:
            return val

    def segment(self, req):

        if self._raw_pc == None:
            print "No PointCloud Received"
            return BBoxSegmentationResponse()

        # cloud properties
        cloud_width = self._raw_pc.width
        cloud_height = self._raw_pc.height
        row_step = self._raw_pc.row_step
        point_step = self._raw_pc.point_step
        x_offset = self._raw_pc.fields[0].offset
        y_offset = self._raw_pc.fields[1].offset
        z_offset = self._raw_pc.fields[2].offset

        # bbox parameters
        box_x0 = int(req.x)
        box_y0 = int(req.y)
        box_width = int(req.width)
        box_height = int(req.height)
        min_height = float(req.min_height)
        max_depth = float(req.max_depth)

        # build uv array for segmentation
        uvs = []
        for u in range(box_x0, box_x0+box_width):
            for v in range(box_y0, box_y0+box_height):
                uvs.append([u,v])


        # find common time for transforming points
        raw_pc = copy.deepcopy(self._raw_pc)

        # find reference frame transformation
        rospy.logwarn(raw_pc.header.frame_id)
        while not rospy.is_shutdown():
            try:
                time = self._tl.getLatestCommonTime(REFERENCE_FRAME, raw_pc.header.frame_id)
                trans, rot = self._tl.lookupTransform(REFERENCE_FRAME, raw_pc.header.frame_id, time)
                transform_mat44 = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
                break
            except:
                print "Waiting for tf between %s and %s" % (raw_pc.header.frame_id, REFERENCE_FRAME)
                continue

        raw_pc.header.stamp = time
        raw_pc.header.frame_id = REFERENCE_FRAME

        segmented_points = []
        points = pcl2.read_points(raw_pc, field_names=('x','y','z'), skip_nans=True, uvs=uvs)

        # build new pc (cutoff the table points)
        # x_sum = 0.
        # y_sum = 0.
        # z_sum = 0.
        x_ps = []
        y_ps = []
        z_ps = []

        heighest_point = -1000.0
        for p in points:
            xyz = tuple(np.dot(transform_mat44, np.array([p[0], p[1], p[2], 1.0])))[:3]
            point = Point(*xyz)

            if point.z > min_height:
                # segmented_points.append([point.x, point.y, point.z])
                # x_sum += point.x
                # y_sum += point.y
                # z_sum += point.z
                x_ps.append(point.x)
                y_ps.append(point.y)
                z_ps.append(point.z)

                if point.z > heighest_point:
                    heighest_point = point.z

        if heighest_point == -1000.0 or max_depth == 0.0:
            x_sum = sum(x_ps)
            y_sum = sum(y_ps)
            z_sum = sum(z_ps)
        else:
            print("Max Depth: {}, Heighest Point: {}".format(max_depth, heighest_point))
            x_sum = sum([p for i,p in enumerate(x_ps) if heighest_point-z_ps[i] < max_depth])
            y_sum = sum([p for i,p in enumerate(y_ps) if heighest_point-z_ps[i] < max_depth])
            z_sum = sum([p for i,p in enumerate(z_ps) if heighest_point-z_ps[i] < max_depth])

        segmented_points = []
        for i in range(len(x_ps)):
            if heighest_point-z_ps[i] < max_depth:
                segmented_points.append([x_ps[i], y_ps[i], z_ps[i]])

        # create segmented PC2
        segmented_pc = pcl2.create_cloud_xyz32(raw_pc.header, segmented_points)


        # compute centroid
        centroid = Point()
        num_points = len(segmented_points)
        if num_points > 0:
            centroid.x = x_sum / len(segmented_points)
            centroid.y = y_sum / len(segmented_points)
            centroid.z = z_sum / len(segmented_points)

        if DEBUG_PC_PUB:
            self._seg_pub.publish(segmented_pc)

            point_stamped = PointStamped()
            point_stamped.header = REFERENCE_FRAME
            point_stamped.point = centroid
            self._centroid_pub.publish(point_stamped)

        # compute object properties
        if num_points > 0:
            segmented_points = np.array(segmented_points)
            max_vertical = segmented_points[:,2].max()
            min_vertical = segmented_points[:,2].min()
            max_horizontal = segmented_points[:,1].max()
            min_horizontal = segmented_points[:,1].min()

            obj_height = max_vertical - min_vertical
            obj_width = max_horizontal - min_horizontal
        else:
            obj_height = -1.0
            obj_width = -1.0

        print "Num Points: %d, Height: %f, Width: %f" % (num_points, obj_height, obj_width)

        return BBoxSegmentationResponse(segmented_pc, centroid, obj_height, obj_width, num_points)

if __name__=='__main__':

    try:
        rospy.init_node('bbox_segmentation_server', anonymous=True)
        bbox_segmentor = BBoxSegmentor('/points', 'bbox_segmentor')

        rospy.spin()

    except rospy.ROSInterruptException:
        pass