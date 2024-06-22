#!/usr/bin/env python3
import rospy

# tf2 and Transformations
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# Camera capture and Image processing
import pyrealsense2 as rs2
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
import open3d as o3d
from ctypes import * # convert float to uint32

from time import sleep

class SavePointcloud(object):
    # To launch just this file (in a brand new terminal): rosrun vision save_point_cloud.py
    # Images saved to /home/drojas/plant_ws/src/wire_manipulation
    def __init__(self, cam_spec : str = "arm_cam"):
        ### Color BGR Image Subscriber
        # Approach: ROS images -> cv bridge -> OpenCV CV images -> OpenCV converts and saves as png/jpg
        self.bgr_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/color/image_raw", Image, self.bgr_callback, (cam_spec), queue_size=1)
        # ^ subscriber to camera publisher, images come back as ROS type Images ----^
        # Currently, we convert them to OpenCV CV type images, but find a way to export either ROS Image or OpenCV image

        ### Depth Image Subscriber
        # Approach: ROS images -> PointCloud2 (nodelet manager in bringup.launch file) -> Open3d with helper function -> Open3d save as .ply
        self.depth_img_sub = rospy.Subscriber(f"/{cam_spec}/rscamera/full_depth_image/points", PointCloud2, self.depth_callback)
        self.segmented_depth_sub = rospy.Subscriber(f"/{cam_spec}/rscamera/depth_image/points", PointCloud2, self.segmented_depth_callback, queue_size=1)

        # Image member variables
        self.bridge_object = CvBridge()

        self.bgr_image = None
        self.full_pointcloud = None
        self.segmented_pointcloud = None

    ### Callbacks to store images
    def bgr_callback(self, data, args):
        try:
            self.bgr_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.full_pointcloud = data
        except CvBridgeError as e:
            print(e)

    def segmented_depth_callback(self, data):
        try:
            self.segmented_pointcloud = data
        except CvBridgeError as e:
            print(e)

    ### Methods for saving individual images
    def save_color_img(self):
        # Save color image in bgr colorspace
        cv2.imwrite("plant_color.jpg", self.bgr_image)

    def save_full_depth_img(self):
        # Save point cloud
        o3d_pc = convertCloudFromRosToOpen3d(self.full_pointcloud)
        o3d.io.write_point_cloud("plant_full_pointcloud.ply", o3d_pc)

    def save_segmented_depth_img(self):
        o3d_pc = convertCloudFromRosToOpen3d(self.segmented_pointcloud)
        o3d.io.write_point_cloud("plant_segmented_pointcloud.ply", o3d_pc)
        
def main():
    rospy.init_node("save_pointcloud",anonymous=True)
    # rospy.sleep(3)
    save_pointcloud = SavePointcloud("arm_cam")
    sleep(5) # sleep to give object time to gather images

    ##### Call saving
    ### Save color BGR image
    save_pointcloud.save_color_img()
    ### Save full depth image as pointcloud in .ply format
    save_pointcloud.save_full_depth_img()

    ### Save segmented depth image as pointcloud in .ply format
    # save_pointcloud.save_segmented_depth_img()





### Helper functions to convert ros image -> open3d
# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

if __name__ == '__main__':
    main()
