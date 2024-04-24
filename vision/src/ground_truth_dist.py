#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs2
from std_srvs.srv import SetBool, SetBoolResponse
import math

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

def monitor_dist():
    """
    Parameters:
        rate_input (float): hertz for rate to check Euclidean distance between marker and executing arm
        end_grasping_arm (str): string specifying whether left or right arm attempted at wire and slipped from
        slip_delta (float): distance in meters of how far marker and arm must be to quantify flag raise
    """
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown(): # loop takes ~0.099 - 0.1 seconds; 99.5ms
        marker_delta_flag = False
        try:
            end_pose = tfBuffer.lookup_transform("world", "mounted_aruco_0", rospy.Time())
            gt = tfBuffer.lookup_transform("world", "ground_truth", rospy.Time())
            euclidean_dist = calc_dist(end_pose, gt)

            print(euclidean_dist)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            continue
        except Exception as e:
            print(e)
            continue
        
        # rate.sleep()

def calc_dist(end_pose, gt_pose):
    end_pose = end_pose.transform.translation
    gt_pose = gt_pose.transform.translation
    return math.sqrt(math.pow(end_pose.x - gt_pose.x, 2) + math.pow(end_pose.y - gt_pose.y, 2) + math.pow(end_pose.z - gt_pose.z, 2))

def main():
    rospy.init_node("ground_truth_dist",anonymous=True)
    rospy.sleep(3)

    monitor_dist()
    
    rospy.spin()

if __name__ == '__main__':
    main()
