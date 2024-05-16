#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
from scipy.ndimage import label

class RGBDSegmentation(object):
    def __init__(self, cam_spec : str = "arm_cam"):

        self.rgb_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/color/image_raw", Image, self.rgb_callback, (cam_spec), queue_size=1)
        # ROS images -> cv bridge -> OpenCV CV images -> ??? -> stl or ply

        # Publishers with segmented image info
        self.segmented_image_pub = rospy.Publisher("segmented_image", Image, queue_size=1)
        self.bbox_image_pub = rospy.Publisher("bbox_image", Image, queue_size=1)

        # Image member variables
        self.bridge_object = CvBridge()

    def rgb_callback(self, data, args):
        try:
            bgr_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        blur_image = cv2.GaussianBlur(bgr_image, (31, 31), 3)

        hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)

        # Segment RGB by Color
        # HSV Ranges for plant green - https://i.stack.imgur.com/gyuw4.png
        green_lower_color = np.array([35, 50,  50]) # rgb values to hsv
        green_upper_color = np.array([80, 255, 255])
        # x, y
        green_mask = cv2.inRange(hsv_image, green_lower_color, green_upper_color)

        # Segment RGB by Coloe
        # HSV Ranges for plant purple - https://i.stack.imgur.com/gyuw4.png
        # purple_lower_color = np.array([100, 0,  200]) # rgb values to hsv
        # purple_upper_color = np.array([145, 40, 255])
        purple_lower_color = np.array([90, 40, 40])  # Adjust values as needed
        purple_upper_color = np.array([160, 180, 235])  # Adjust values as needed
        # purple_lower_color = np.array([100, 30, 30])  # Adjust values as needed original values
        # purple_upper_color = np.array([160, 120, 200])  # Adjust values as needed

        # x, y
        purple_mask = cv2.inRange(hsv_image, purple_lower_color, purple_upper_color)

        # green_segmented_image = cv2.bitwise_and(bgr_image, bgr_image, mask=green_mask)
        # purple_segmented_image = cv2.bitwise_and(bgr_image, bgr_image, mask=purple_mask)

        combined_mask = cv2.bitwise_or(green_mask, purple_mask)
        # combined_mask

        segmented_image = cv2.bitwise_and(bgr_image, bgr_image, mask=combined_mask)

        min_col, min_row, max_col, max_row = self.find_bounding_box(combined_mask)
        bbox_img = self.draw_bounding_box(bgr_image, min_col, min_row, max_col, max_row)

        print(self.offset_threshold(min_col, min_row, max_col, max_row))



        # Publish the green segmented image
        try:
            bbox_image_msg = self.bridge_object.cv2_to_imgmsg(bbox_img, encoding="bgr8")
            self.bbox_image_pub.publish(bbox_image_msg)
        except CvBridgeError as e:
            print(e)

        try:
            segmented_image_msg = self.bridge_object.cv2_to_imgmsg(segmented_image, encoding="bgr8")
            self.segmented_image_pub.publish(segmented_image_msg)
        except CvBridgeError as e:
            print(e)

    def find_bounding_box(self, binary_mask):
        # Apply morphological closing to connect nearby regions
        kernel = np.ones((51, 51), np.uint8)
        closed_mask = cv2.morphologyEx(binary_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        labeled_mask, num_labels = label(closed_mask)
        regions, counts = np.unique(labeled_mask, return_counts=True)
        
        # Skip the background label (0) and find the most dense region
        max_density_region = regions[np.argmax(counts[1:]) + 1]
        
        region_indices = np.where(labeled_mask == max_density_region)
        min_row, min_col = np.min(region_indices[0]), np.min(region_indices[1])
        max_row, max_col = np.max(region_indices[0]), np.max(region_indices[1])
        
        return min_col, min_row, max_col, max_row
        

    def draw_bounding_box(self, image, min_col, min_row, max_col, max_row):
        bbox_image = cv2.rectangle(image, (min_col, min_row), (max_col, max_row), (0, 255, 0), 2)
        return bbox_image
    
    def offset_threshold(self, min_col, min_row, max_col, max_row):
        # Image size is 1280w x 720h
        min_horizontal_offset_threshold = 1280 * 0.01 # = 25.6
        min_vertical_offset_threshold = 720 * 0.01 # = 14.4
        max_horizontal_offset_threshold = 1280 * 0.15 # = 384
        max_vertical_offset_threshold = 720 * 0.15 # = 216

        left_offset= min_col
        right_offset= 1280-max_col
        top_offset=min_row
        bottom_offset=720-max_row
        
        min_left_bool = left_offset > min_horizontal_offset_threshold
        max_left_bool = left_offset < max_horizontal_offset_threshold
        min_right_bool = right_offset > min_horizontal_offset_threshold
        max_right_bool = right_offset < max_horizontal_offset_threshold
        min_top_bool = top_offset > min_vertical_offset_threshold
        max_top_bool = top_offset < max_vertical_offset_threshold
        min_bottom_bool = bottom_offset > min_vertical_offset_threshold
        max_bottom_bool = bottom_offset < max_vertical_offset_threshold

        total_offset_bool = (min_left_bool and max_left_bool) and (min_right_bool and max_right_bool) and (min_top_bool and max_top_bool) and (min_bottom_bool and max_bottom_bool)

        offset_result = {}
        if not total_offset_bool:
            if not min_left_bool:
                # print('adjust arm to address left side being too close')
                offset_result["left_close"] = left_offset
            elif not max_left_bool:
                # print('adjust arm to address left side being too far')
                offset_result["left_far"] = left_offset

            if not min_right_bool:
                # print('adjust arm to address right side being too close')
                offset_result["right_close"] = right_offset
            elif not max_right_bool:
                print('adjust arm to address right side being too far')
                offset_result["right_far"] = right_offset

            if not min_top_bool:
                # print('adjust arm to address top side being too close')
                offset_result["top_close"] = top_offset
            elif not max_top_bool:
                # print('adjust arm to address top side being too far')
                offset_result["top_far"] = top_offset

            if not min_bottom_bool:
                # print('adjust arm to address bottom side too close')
                offset_result["bottom_close"] = bottom_offset
            elif not max_bottom_bool:
                # print('adjust arm to address bottom side too far')
                offset_result["bottom_far"] = bottom_offset

        print(f"top: {top_offset} bottom: {bottom_offset}")
        return offset_result
        
        



def main():
    rospy.init_node("rgbd_seg",anonymous=True)
    # rospy.sleep(3)
    mounted_seg_object = RGBDSegmentation("arm_cam")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
