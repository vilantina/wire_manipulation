#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
from geometry_msgs.msg import Pose

class RGBSegmentation(object):
    def __init__(self):
        # Subscribers to Camera
        # self.aligned_depth_rgb_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_depth_data,queue_size=1)
        # self.rgb_img_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.rgb_callback,queue_size=1)
        # self.depth_img_camera_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback,queue_size=1)
        
        # Compressed GTRI Bag
        # self.aligned_depth_rgb_sub = rospy.Subscriber("/cam_0/aligned_depth_to_color/image_raw/compressed", CompressedImage, self.get_depth_data,queue_size=1)
        # self.rgb_img_sub = rospy.Subscriber("/cam_0/color/image_raw/compressed",CompressedImage, self.rgb_callback,queue_size=1)
        # self.depth_img_camera_info = rospy.Subscriber("/cam_0/depth/camera_info",CameraInfo, self.depth_cam_info_callback,queue_size=1)

        # Uncompressed GTRI Bag
        self.aligned_depth_rgb_sub = rospy.Subscriber("/cam_0/aligned_depth_to_color/image_raw", Image, self.get_depth_data,queue_size=1)
        self.rgb_img_sub = rospy.Subscriber("/cam_0/color/image_raw", Image, self.rgb_callback,queue_size=1)
        self.depth_img_camera_info = rospy.Subscriber("/cam_0/depth/camera_info", CameraInfo, self.depth_cam_info_callback,queue_size=1)


        # Publishers with segmented image info
        self.image_pub = rospy.Publisher("/rs_segmented_image", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher("/seg_depth/image_raw", Image, queue_size=1)
        self.depth_img_cam_info_pub = rospy.Publisher("/seg_depth/camera_info", CameraInfo, queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()
        self.seg_depth_img = Image()

    def preview_img(self, img):
        preview = cv2.resize(img, (1280,720), interpolation=cv2.INTER_LINEAR)
        cv2.imshow('cv_image', preview) 
        cv2.waitKey(1)

    def rgb_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # cv_image = self.bridge_object.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # Segment RGB by Coloe
        lower_color = np.array([ 124, 72, 47])
        upper_color = np.array([179, 255, 255])
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        new_img = cv2.bitwise_and(cv_image, cv_image, mask = mask ) # pass

        # dilation
        kernel = np.ones((5,5), np.uint8)
        img_dilation = cv2.dilate(new_img, kernel, iterations=1)
        img_dilation_gray = cv2.cvtColor(img_dilation,cv2.COLOR_BGR2GRAY) # pass
        
        # find largest contour
        contours, hierch = cv2.findContours(img_dilation_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        largest_area = sorted(contours, key= cv2.contourArea)
        mask2 = np.zeros(img_dilation_gray.shape, np.uint8)

        filtered_wire = cv2.drawContours(mask2,[largest_area[-1]], 0, (255,255,255,255), -1) # pass

        # erosion
        img_erosion = cv2.erode(filtered_wire, kernel, iterations=2) # pass

        # Get copy of depth image
        depth = self.depth_data
        depth_copy = depth.copy()

        # use segmented RGB image as mask for depth image
        new_depth_img = cv2.bitwise_and(depth_copy, depth_copy, mask = img_erosion )

        # Define Camera info for publish
        cam_info = CameraInfo()
        cam_info.header.stamp = rospy.Time.now()
        cam_info.header.frame_id = "cam_0_color_optical_frame"
        cam_info.height = 1080
        cam_info.width = 1920
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = self.depth_cam_info.D
        cam_info.K = self.depth_cam_info.K
        cam_info.R = self.depth_cam_info.R
        cam_info.P = self.depth_cam_info.P
        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi = self.depth_cam_info.roi

        # Segmented RGB Image
        segmented_img = self.bridge_object.cv2_to_imgmsg(new_img, encoding="passthrough") # ERROR not numpy array or scalar expected
        segmented_img.header.frame_id = "cam_0_color_optical_frame" 

        # Segmented Depth Image
        #ENCODING ISSUE IS HERE, WE USE 16UC1 vs 8UC1 issue
        self.seg_depth_img = self.bridge_object.cv2_to_imgmsg(new_depth_img, encoding="16UC1") # ERROR not numpy array or scalar expected
        self.seg_depth_img.header.stamp = cam_info.header.stamp
        self.seg_depth_img.header.frame_id = "cam_0_color_optical_frame"

        # Publish
        self.image_pub.publish(segmented_img)
        self.depth_image_pub.publish(self.seg_depth_img)
        """
        Nodelet manager in bringup.launch file subscribes to this in its image_rect subscribed topic
        <remap from="image_rect" to="/seg_depth/image_raw"/>
        Commenting out this line from the launch file removes the encoding 8UC1 error.
        This topic is created and published here in rgb_segmentation. 
        This means the encoding issue is somewhere in how self.seg_depth_img is created and ultimately published.
        Currently in 8UC1, need to get it to
        """
        self.depth_img_cam_info_pub.publish(cam_info)

    def get_depth_data(self,data):
        cv_depth_image = self.bridge_object.imgmsg_to_cv2(data)
        # cv_depth_image = self.bridge_object.compressed_imgmsg_to_cv2(data)
        self.depth_data = cv_depth_image

    def depth_cam_info_callback( self,msg):
        self.depth_cam_info = msg


def main():
    rospy.init_node("seg_node",anonymous=True)
    rospy.sleep(3)
    seg_object = RGBSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
