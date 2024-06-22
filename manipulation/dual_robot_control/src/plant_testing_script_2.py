#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from time import sleep
from std_msgs.msg import Bool, Time
from colorama import Fore
from std_srvs.srv import SetBool

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/plant_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

def set_pose_spec_service(value : Bool):
     rospy.wait_for_service("/set_pose_spec")
     try:
         set_pose_spec = rospy.ServiceProxy('/set_pose_spec', SetBool)
         response = set_pose_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    robot_control = RobotControl()

    # GRASPING_ARM    = "right"
    # GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"
    SEARCHING_ARM   = "left"
    # SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"
    # arm_ids = ["left","right"]

    joint_goal_1 = [-65, 4, 49, -99, -87, 41] # original 2nd pose
    status = robot_control.move_to_joint_goal(SEARCHING_ARM, [x * np.pi / 180 for x in joint_goal_1])
    # wait again for the arm to move
    sleep(10)

    # save again
    # print(success, message)
    success, message = set_pose_spec_service(True) # Swap back to rear cam
    sleep(5)

    # rospy.spin()
