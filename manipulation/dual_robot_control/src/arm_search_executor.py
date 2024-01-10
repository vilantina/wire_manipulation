#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool
from colorama import Fore
from std_srvs.srv import SetBool

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/dlo_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

# Client call to sleep specified arm
def sleep_arm(robot_):
    rospy.wait_for_service('sleep_arm_service')
    
    sleep_arm_input = rospy.ServiceProxy('/sleep_arm_service', GraspObject)
    req = GraspObjectRequest()
    pose = geometry_msgs.msg.Pose()
    pose.position = geometry_msgs.msg.Point(0,0,0)
    pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,0)
    req.robot = 'left'
    req.object_grasp_pose = pose
    response = sleep_arm_input(req)
    
# Client call to swap ML camera specification 
def set_cam_spec_service(value : Bool):
     rospy.wait_for_service("/set_cam_spec")
     try:
         set_cam_spec = rospy.ServiceProxy('/set_cam_spec', SetBool)
         response = set_cam_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def check_frame_exists(target, source):
    """
    Check if a given frame exists
    """
    listener = tf.TransformListener()
    return listener.waitForTransform(source, target, rospy.Time(), rospy.Duration(1.0))

#*** Node Starts Here ***#
if __name__ == "__main__":
    ### OVERALL SEARCH DEMO
    # Search algorithm is demonstrated with the following steps:
    # 0. Initiate unplugging of the connector from mock battery ORU
    # 1. Purposefully induce an engineered slip to require anomaly detection and correction
    # 2. Detect slip using Euclidean distance between the connector and grasping arm's end effector
    # 3. Flag the slip and begin searching for lost connector end with spiral search algorithm
    #    3a. Move searching arm to first search target
    #    3b. Conduct search on search target
    #    3c. If nothing found, move search target and arm and loop
    # 4. If connector found, transform frame of connector from searching arm to grasping arm and initiate retrieval
    #    Else, begin search elsewhere in environment
    # 5. Return retrieved connector end to originally intending stowing point 

    rospy.init_node('listener', anonymous=True)
    robot_control = RobotControl()

    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"
    SEARCHING_ARM   = "left"
    SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"
    arm_ids = ["left","right"]
    
    # Send object grasp robot to pre-grasp, encountering wire
    joint_goal0 = [41, 1, -2, 90, -43, 3] # start
    joint_goal0_5 = [57, -14, 14, 89, -60, 4]
    joint_goal1 = [21, -13, 15, 81, -19, 9] # unplug
    # Bring in front of camera, slip
    joint_goal2 = [36, 13, -41, 153, -90, -53] # slip enroute angled down
    joint_goal3 = [32, -7, -3, -78, 35, 74] # final stowing
    


    ### START ROUTINE for full demonstration at annual review

    # ##  Initialize arms; Sleep, open grippers, and ready pose
    # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots")
    # for arm in arm_ids: 
    #     status = robot_control.move_to_target(arm, 'sleep')
    #     status = robot_control.set_gripper(arm, "open")

    # ##  Move grasping arm to wire end and unplug
    # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Moving grasping arm to connector end and unplug from battery ORU")
    # status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0_5])
    # status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0])
    # ## Grip and unplug
    # status = robot_control.set_gripper(GRASPING_ARM, "close")
    # status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal1])
    # status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0])
    # status = robot_control.set_gripper(GRASPING_ARM, "open")
                       
    ### START demo as wire is grasped, earliest wire could slip
    print("STATUS: Begin Scenario C4 with ")
    commands = [ # Commands from A1
        # Send to stowing point, engineering slip
        "status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal2])",
    ]
    for command in commands:
        # exec(command)
        # slip_flag = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
        slip_flag = True
        if (slip_flag): # If slip detected, move arm to retrieve wire
            print("STATUS: Slip detected, initiate retrieval")
            # sleep(10) # wait 5 real time seconds for slipped wire to settle

            # First use rear camera to detect existence and location of connector
            try:
                check_frame_exists("line_grasp_mounted_cam", "world")
                status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
                status = robot_control.set_gripper(GRASPING_ARM, "open")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                # If rear camera fails, switch to search pattern
                # success, message = set_cam_spec_service(True) # Swap to arm cam
                ### Initiate Spiral searching
                # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear camera view attempt failed, initiate search routine")
                # # Initiate search algorithm
                # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiate search routine with arm cam")

                # success, message = set_cam_spec_service(True) # Swap to arm cam
                # sleep(2.5)

                # # Begin search
                # searchRoutine = SC.SearchRoutine("left", "right")
                # search_result = searchRoutine.search(check_subnodes=True)

                # if search_result:
                #     print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Search successful, send grasping arm for retrieval")
                #     # send right arm to aruco left arm found
                #     sleep(2.5)
                #     status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_arm_cam")
                #     status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_arm_cam")
                #     # Search done, return view to rear cam
                #     success, message = set_cam_spec_service(False) # Swap back to rear cam
                #     sleep(2.5)  
            # finally:
            #     # Restore connector to final pose
            #     status = robot_control.set_gripper(GRASPING_ARM, "close")
            #     status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal3])  

    ### END SCENARIO C4

    # rospy.spin()
