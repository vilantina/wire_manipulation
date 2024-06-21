#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose
import tf
import tf2_ros
import tf.transformations as tft
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse

class PoseTransformationCalculator(object):
    def __init__(self):
        # Subscribers to Pose publisher for arm camera
        # self.cam_pose_sub = rospy.Subscriber('/current_pose_topic', Pose, self.current_pose_callback)

        # Saved poses as 4x4 transformation matrices
        self.poses = [] # {}
        self.pose_counter = 0

    def current_pose_save(self):
        global POSE_CHANGED
        if not POSE_CHANGED:
            return
    
        # data comes in as /tf information, extract pose info from this
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(1)
        try:
            pose_transform = tfBuffer.lookup_transform('d435i_infra2_frame', 'world', rospy.Time(0), rospy.Duration(5))
            # pose_transform = tfBuffer.lookup_transform('world', 'd435i_link', rospy.Time(0))
            new_pose = Pose()
            new_pose.position = pose_transform.transform.translation
            new_pose.orientation = pose_transform.transform.rotation

            self.poses.append(self.pose_to_matrix(new_pose))
            POSE_CHANGED = False
            # self.pose_counter += 1
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Transform lookup failed.")
            print(e)
        # pose = data.something, want d435i_color_optical_frame
        # """
        # Convert a Pose message to a 4x4 transformation matrix
        # """
        # position = pose.position
        # orientation = pose.orientation

        # translation_matrix = tft.translation_matrix([position.x, position.y, position.z])
        # rotation_matrix = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # transformation_matrix = np.dot(translation_matrix, rotation_matrix)
            
    def pose_to_matrix(self, pose):
        """
        Convert a Pose message to a 4x4 transformation matrix.
        """
        position = pose.position
        orientation = pose.orientation

        # Create a translation matrix from the position
        translation_matrix = tft.translation_matrix([position.x, position.y, position.z])
        # Create a rotation matrix from the quaternion
        rotation_matrix = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # to_euler = tft.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # rotation_matrix = tft.quaternion_matrix(to_euler)
        # print("Translation Matrix\n",translation_matrix)
        # print("Rotation Matrix\n",rotation_matrix)
        # rotation_matrix = tft.euler_matrix(to_euler)

        # Combine translation and rotation into a single transformation matrix
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        
        return transformation_matrix

    def compute_transform_difference(self, pose_0_index=0, pose_1_index=1):
        """
        Compute the transformation matrix representing the 
        positional and orientation difference between two poses.
        """
        # initial and final matrices will be stored in self.poses
        # Compute the relative transformation matrix
        # print(self.poses[pose_0_index], self.poses[pose_1_index])
        relative_transform = np.matmul((self.poses[pose_0_index]), np.linalg.inv(self.poses[pose_1_index]))
        
        return relative_transform
    
    def get_poses(self):
        return self.poses

POSE_CHANGED = False
def set_pose_spec(request):
    """
    Switches between last pose seen, specifies a new pose is to be saved
    """
    global POSE_CHANGED
    # Service sets to True, object sets to False meaning its been saved
    POSE_CHANGED = bool(request.data)
    return SetBoolResponse(request.data, f"Pose change variable changed to {POSE_CHANGED}")

def main():
    rospy.init_node("pose_transformation_calc",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)
    SAVED = False

    set_pose_spec_service = rospy.Service("/set_pose_spec", SetBool, set_pose_spec)
    global POSE_CHANGED
    pose_tracker = PoseTransformationCalculator()
    while not rospy.is_shutdown():
        if POSE_CHANGED == True:
            pose_tracker.current_pose_save()
        # print(len(pose_tracker.get_poses()))

        # print(len(pose_tracker.get_poses()))
        if not SAVED and len(pose_tracker.get_poses()) >= 2:
            # print(len(pose_tracker.get_poses()))
            # print(pose_tracker.get_poses())
            # diff calc
            transformation_mat = pose_tracker.compute_transform_difference(0,1)

            text_file = open("transformation_mat.txt", "w")
            text_file.write(str(transformation_mat))
            text_file.close()

            SAVED = False

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
