#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs2
from std_srvs.srv import SetBool, SetBoolResponse
import cv2
from collections import defaultdict
import numpy as np
import tf_conversions
import math
import cam_calibration

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

class PlantVision:
    def __init__(self, matrix_coefficients, distortion_coefficients):
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
        # Subscribers to Camera
        self.aligned_depth_rgb_sub = rospy.Subscriber("/arm_cam/camera/aligned_depth_to_color/image_raw", Image, self.get_depth_data,queue_size=1)
        self.rgb_img_sub = rospy.Subscriber("/arm_cam/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        self.depth_img_camera_info = rospy.Subscriber("/arm_cam/camera/aligned_depth_to_color/camera_info", CameraInfo, self.depth_cam_info_callback,queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.seg_depth_img = Image()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()

        # Image Coefficients
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients

        # Track result vectors for each id tag found
        self.marker_dict = defaultdict(dict)
        # tvec is 3d position difference between the camera and the marker
        # rvec is Rodriguez's angles between the camera and marker center

    def track_callback(self, data):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # ret, frame = CAMERA_SRC.read() # If not using ROS

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters =  cv2.aruco.DetectorParameters()
        # lists of ids and the corners belonging to each id
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters)
        
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.0508, self.matrix_coefficients, # this marker is 2 inches; conv to meters
                                                                        self.distortion_coefficients)
                (rvec - tvec).any()  # Remove numpy value array error
                self.marker_dict[i]["tvec"] = tvec
                self.marker_dict[i]["rvec"] = rvec

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "b_bot_ee_arm_link"
                t.child_frame_id = "arm_aruco_{}".format(i)
                t.transform.translation.x = tvec.reshape(3)[2]
                t.transform.translation.y = -tvec.reshape(3)[0]
                t.transform.translation.z = -tvec.reshape(3)[1]
                
                rot_mat = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
                rot_mat[:3, :3], _ = cv2.Rodrigues(rvec)
                e = tf_conversions.transformations.euler_from_matrix(rot_mat)
                q = tf_conversions.transformations.quaternion_from_euler(e[0], e[1]-math.pi/2, e[2])
                # q = tf_conversions.transformations.quaternion_from_euler(e[0], e[1], e[2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                
                br.sendTransform(t)
                
                cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, .2) 
        # else:
        #     print("ARUCO NO LONGER VISIBLE")


        # Display the resulting frame
        resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
        cv2.imshow('Arm Mounted Camera', resized_frame) 
        cv2.waitKey(1)

    def get_depth_data(self,data):
        cv_depth_image = self.bridge_object.imgmsg_to_cv2(data)
        self.depth_data = cv_depth_image

    def depth_cam_info_callback(self, msg):
        self.depth_cam_info = msg
        # print(pc)

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

    def transform_aruco_rotation(child_name: str, source: str, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "{}_{}".format(child_name, source)

        t.transform.translation.x = ori_adj[0] # Offset arm to right by value meters
        t.transform.translation.y = ori_adj[1]
        t.transform.translation.z = ori_adj[2] # Too close to wall, move back .05m

        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

def main():
    rospy.init_node("depth_tracker",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)

    # plant_vision = PlantVision()
    # Define calibration object to hold and store points
    calibration = cam_calibration.CameraCalibration()

    # Defiine arguments to pass to calibrate() parameters
    directory_path = "/home/drojas/dlo_ws/src/wire_manipulation/vision/resources/calibration/arm_cam/*"
    img_file_prefix = "img_"
    img_format = ".jpg"
    square_size = 0.0127 # in meters; each square is 0.5inch
    height = 20-1 # squares high
    width = 20-1 # squares across

    calibration_matrices = calibration.calibrate(directory_path, img_file_prefix, img_format, square_size, height, width)
    tracker = PlantVision(calibration_matrices[1], calibration_matrices[2])

    while not rospy.is_shutdown():
        tracker.transform_aruco_rotation("adj", "mounted_aruco_0", [0, math.pi/2, 0], [WIRE_OFFSET, 0, 0.05])

    rospy.spin()

if __name__ == '__main__':
    main()
