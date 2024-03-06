#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
import colorsys

class RGBDSegmentation(object):
    def __init__(self, cam_spec : str = "mounted_cam"):
        ### Camera Subscribers and Publishers
        self.aligned_depth_rgb_sub = rospy.Subscriber(f"/{cam_spec}/camera/aligned_depth_to_color/image_raw_throttled", Image, self.get_depth_data,queue_size=1)
        self.rgb_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/color/image_raw_throttled", Image, self.rgb_callback, (cam_spec), queue_size=1)
        self.depth_img_camera_info = rospy.Subscriber(f"/{cam_spec}/camera/aligned_depth_to_color/camera_info_throttled",CameraInfo, self.depth_cam_info_callback,queue_size=1)

        self.plant_rgb_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/color/image_raw", Image, self.plant_rgb_callback, (cam_spec), queue_size=1)


        # Publishers with segmented image info
        self.image_pub = rospy.Publisher(f"/{cam_spec}/rs_segmented_image", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher(f"/{cam_spec}/seg_depth/image_raw", Image, queue_size=1)
        self.depth_img_cam_info_pub = rospy.Publisher(f"/{cam_spec}/seg_depth/camera_info", CameraInfo, queue_size=1)
      
        # Image member variables
        self.bridge_object = CvBridge()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()
        self.seg_depth_img = Image()

    def plant_rgb_callback(self, data, args):
        try:
            print(data)
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="rgb8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        view_analysis_count = { # dictionary
            "top" : 0,
            "bottom" : 0,
            "left": 0,
            "right": 0,
        }

        # print(data.height, data.width) # 720 1280
        for w_i in range(data.width): #1280px
            if w_i == 0 or w_i == 1279:
                # check the whole height, 
                # meaning we for loop down w
                pass
                # for w in data.height:
                #     b, g, r = cv_image[y, x]
            else:
                # we only check w's 0 or 719 at the current w_i
                # print(w_i)

                # top 
                r, g, b = cv_image[0, w_i] # [y, x]
                # print(r,g,b)
                # # check this bgr
                # top_color_check_bool = self.check_color(b,g,r)
                # # if the check is green or purple
                # if top_color_check_bool == True:
                #     view_analysis_count["top"] = view_analysis_count["top"] + 1 # increase counter for top

                # # bottom
                # b, g, r = cv_image[719, w_i]
                # # check this bgr
                # bottom_color_check_bool = self.check_color(b,g,r)
                # # if the check is green or purple
                # if bottom_color_check_bool == True:
                #     view_analysis_count["bottom"] = view_analysis_count["bottom"] + 1 # increase counter for top

        threshold_value = 25
        threshold_check = any(value > threshold_value for value in view_analysis_count.values())
        
        # print(threshold_check)
        # print(view_analysis_count)
        if threshold_check == True:
            # let us know, determine the direction for the arm to move
            pass

    def check_color(self, b,g,r):
        threshold = 200
        b_check = b >= threshold # and b <= 255
        g_check = g >= threshold
        r_check = r >= threshold
        return b_check and g_check and r_check
    
        # we receive the bgr from the image, 
        # now check if that bgr value is within a
        # range of green or purple we set
        # print(colorsys.hsv_to_rgb(0,0,0))
        # print(colorsys.hsv_to_rgb(10,10,10/255))

        # Define the lower and upper bounds of the green color range in BGR format
        # lower_green = np.array([0, 0, 50], dtype=np.uint8)
        # upper_green = np.array([180, 50, 255], dtype=np.uint8)
                                # 180, 255, 255
        
        # our bgr value
        # bgr_value = np.array([b, g, r], dtype=np.uint8)
        # bgr_color = np.uint8([[[b, g, r]]])

        # hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)


        # Create a mask to check if the BGR value is within the green range
        # mask = cv2.inRange(hsv_color, lower_green, upper_green)
        # print(mask)
        # return mask[0][0] == 255 # if mask value is 255, BGR value is within the green range
            


    def rgb_callback(self, data, args):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # Segment RGB by Coloe
        # HSV Ranges for plant green - https://i.stack.imgur.com/gyuw4.png
        green_lower_color = np.array([35, 50,  50]) # rgb values to hsv
        green_upper_color = np.array([80, 255, 255])
        # x, y
        green_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(green_hsv, green_lower_color, green_upper_color)

        # Segment RGB by Coloe
        # HSV Ranges for plant purple - https://i.stack.imgur.com/gyuw4.png
        # purple_lower_color = np.array([100, 0,  200]) # rgb values to hsv
        # purple_upper_color = np.array([145, 40, 255])
        purple_lower_color = np.array([170, 100,  50]) # rgb values to hsv
        purple_upper_color = np.array([180, 255, 255])
        # x, y
        purple_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        purple_mask = cv2.inRange(purple_hsv, purple_lower_color, purple_upper_color)

        combined_mask = cv2.bitwise_or(purple_mask, green_mask)
        ### Why cv2.bitwise_and() doesn't work:
        # not seeing any output because there are no overlapping regions between the two masks,
        # meaning there are no pixels that are purple or green in both masks (each mask is only one color).


        new_img = cv2.bitwise_and(cv_image, cv_image, mask = combined_mask )
        # # Display the resulting frame
        # resized_frame = cv2.resize(new_img, (0,0), fx=0.80, fy=0.80)
        # cv2.imshow('new_img', resized_frame) 
        # cv2.waitKey(1)

        # dilation
        kernel = np.ones((5,5), np.uint8)
        img_dilation = cv2.dilate(new_img, kernel, iterations=1)
        img_dilation_gray = cv2.cvtColor(img_dilation,cv2.COLOR_BGR2GRAY)
        
        # # find largest contour
        contours, hierch = cv2.findContours(img_dilation_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        largest_area = sorted(contours, key= cv2.contourArea)
        mask2 = np.zeros(img_dilation_gray.shape, np.uint8)

        try:
            filtered_wire = cv2.drawContours(mask2,[largest_area[-1]], 0, (255,255,255,255), -1)
        except IndexError:
            # No contours/pc of segmented wire drawn because no view of red wire
            return

        # erosion
        img_erosion = cv2.erode(filtered_wire, kernel, iterations=2)

        # Get copy of depth image
        depth = self.depth_data
        depth_copy = depth.copy()

        # use segmented RGB image as mask for depth image
        try:
            new_depth_img = cv2.bitwise_and(depth_copy, depth_copy, mask = img_erosion )
        except:
            return

        optical_frame_prefix = "d435i" if args == "arm_cam" else "d415"
        # Define Camera info for publish
        cam_info = CameraInfo()
        cam_info.header.stamp = rospy.Time.now()
        cam_info.header.frame_id = f"{optical_frame_prefix}_color_optical_frame"
        cam_info.height = 720
        cam_info.width = 1280
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = self.depth_cam_info.D
        cam_info.K = self.depth_cam_info.K
        cam_info.R = self.depth_cam_info.R
        cam_info.P = self.depth_cam_info.P
        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi = self.depth_cam_info.roi

        # Segmented RGB Image
        segmented_img = self.bridge_object.cv2_to_imgmsg(new_img,"passthrough")
        segmented_img.header.frame_id = f"{optical_frame_prefix}_color_optical_frame"
        # print(new_img)

        # Segmented Depth Image
        self.seg_depth_img = self.bridge_object.cv2_to_imgmsg(new_depth_img)
        self.seg_depth_img.header.stamp = cam_info.header.stamp
        self.seg_depth_img.header.frame_id = f"{optical_frame_prefix}_color_optical_frame"

        # # Display the resulting frame
        # resized_frame = cv2.resize(new_img, (0,0), fx=0.80, fy=0.80)
        # cv2.imshow('Preview', segmented_img) 
        # cv2.waitKey(1)
    
        # Publish
        self.image_pub.publish(segmented_img)
        self.depth_image_pub.publish(self.seg_depth_img)
        self.depth_img_cam_info_pub.publish(cam_info)

    def get_depth_data(self,data):
        cv_depth_image = self.bridge_object.imgmsg_to_cv2(data)
        self.depth_data = cv_depth_image

    def depth_cam_info_callback( self,msg):
        self.depth_cam_info = msg


def main():
    rospy.init_node("rgbd_seg",anonymous=True)
    rospy.sleep(3)
    mounted_seg_object = RGBDSegmentation("mounted_cam")
    arm_seg_object = RGBDSegmentation("arm_cam")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
