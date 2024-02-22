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

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

class PlantVision:
    def __init__(self):
        # Subscribers to Camera
        self.depth_cam_info = rospy.Subscriber("/mounted_cam/camera/depth/color/points", PointCloud2, self.depth_cam_info_callback)
        # self.segmented_depth_sub = rospy.Subscriber(f"/{cam_spec}/rscamera/depth_image/points", PointCloud2, self.segmented_depth_callback, queue_size=1)

        #### HW Example:
        # 0.) Setup Publisher with unique topic, and publish pc data that is consumed by subscriber callback
        # self.example_pub = rospy.Publisher(...)

        # 1.) In terminal, see that your topic exist
        # rostopic list
        #
        # 2.) in rviz, see that you can visualize data from your topic


        
        
        
        #####
        # Init Open3D window 
        # self.old_pc = PointCloud2(np.array([]))
        # self.registered_pc_pub = Publisher(...)


    # def registation(self, old, new):
    #     pass

    def depth_cam_info_callback(self, pc):
        print(pc)

        ######
        # publish pc on self.example_pub topic

        #### registration
        # new_pc = self.registation(self.old_pc, pc)
        #
        # 

        ####
        # self.registered_pc_pub.publish(new_pc)

        # self.old_pc = new_pc



        # try:
        #     if self.intrinsics:
        #         return
        #     self.intrinsics = rs2.intrinsics()
        #     self.intrinsics.width = cameraInfo.width
        #     self.intrinsics.height = cameraInfo.height
        #     self.intrinsics.ppx = cameraInfo.K[2]
        #     self.intrinsics.ppy = cameraInfo.K[5]
        #     self.intrinsics.fx = cameraInfo.K[0]
        #     self.intrinsics.fy = cameraInfo.K[4]
        #     if cameraInfo.distortion_model == 'plumb_bob':
        #         self.intrinsics.model = rs2.distortion.brown_conrady
        #     elif cameraInfo.distortion_model == 'equidistant':
        #         self.intrinsics.model = rs2.distortion.kannala_brandt4
        #     self.intrinsics.coeffs = [i for i in cameraInfo.D]

        # except CvBridgeError as e:
        #     print(e)
        #     return

def main():
    rospy.init_node("depth_tracker",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)

    plant_vision = PlantVision()
    # while not rospy.is_shutdown():
    #     pass
        
    #     rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
