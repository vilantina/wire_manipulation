#!/usr/bin/env python3
from time import sleep
import rospy

from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

import numpy as np
import open3d as o3d

import ros_numpy
from stl import mesh

from mpl_toolkits import mplot3d
from matplotlib import pyplot

# Transform publishing
from tf.transformations import quaternion_from_euler, quaternion_about_axis, euler_from_quaternion
import tf2_geometry_msgs, geometry_msgs.msg
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

rospy.init_node("match_connector_cam",anonymous=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def transform_connector_match_cam(child_name: str, source: str, cam_spec, pos_adj, ori_adj):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "connector_{}".format(cam_spec)

        t.transform.translation.x = ori_adj[0]
        t.transform.translation.y = ori_adj[1] 
        t.transform.translation.z = ori_adj[2] #+ 0.25

        transform = None
        try:
            transform = tf_buffer.lookup_transform(source, cam_spec, rospy.Time())

        except Exception as e:
            print(e)
        if not transform:
             return
        
        # Align Z axes here from grasp frame to camera frame
        rotation = transform.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        q = quaternion_from_euler(0, yaw, 0) # either pitch or yaw, yaw looks more correct? test with arm cam

        # Set new axes
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Apply the new transformation to frame A
        br.sendTransform(t)

def main():
    rate = rospy.Rate(60)
    rear_cam_spec = "mounted_cam"
    arm_cam_spec = "arm_cam"
    # CAM_SPEC = "mounted"
    # cam_spec_name = "d415_color_frame" if 

    while not rospy.is_shutdown():
        transform_connector_match_cam("adj_grasp_mounted_cam", "line_grasp_mounted_cam", "d415_color_frame", [0,0,0], [0, 0, 0, 1])

if __name__ == '__main__':
    main()
