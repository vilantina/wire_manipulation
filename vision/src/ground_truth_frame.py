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

    
def generate_ground_truth(pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = f"ground_truth"

    t.transform.translation.x = pos_adj[0]
    t.transform.translation.y = pos_adj[1]
    t.transform.translation.z = pos_adj[2]
    
    q = quaternion_from_euler(ori_adj[0], ori_adj[1], ori_adj[2])

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

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
            end_pose = tfBuffer.lookup_transform("world", "line_grasp_mounted_cam", rospy.Time())
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

def calc_dist(self, end_pose, arm_pose):
    end_pose = end_pose.transform.translation
    arm_pose = arm_pose.pose.position
    return math.sqrt(math.pow(end_pose.x - arm_pose.x, 2) + math.pow(end_pose.y - arm_pose.y, 2) + math.pow(end_pose.z - arm_pose.z, 2))

def main():
    rospy.init_node("ground_truth_frame",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        generate_ground_truth([0.32, 0.07, 0.21], [0,0,0,0])
        
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
