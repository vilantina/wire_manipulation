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
from sensor_msgs.msg import PointCloud2

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Diego's custom libs
from wire_modeling.wire_sim import Collisions,TargetObject,WireModel,WireSim
from wire_modeling_msgs.srv import *
# from dual_robot_msgs.srv import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox, rotm

### Initiating robot arm controls
# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/dlo_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

CURR_REAR_TIMESTAMP = None
def calc_active_callback(ts):
    global CURR_REAR_TIMESTAMP
    CURR_REAR_TIMESTAMP = ts

REAR_TIMESTAMP_SUB = rospy.Subscriber("/rear_timestamp", Time, calc_active_callback, queue_size=1)
    
### Diego's functions for brushing wire
# returns a pointcloud instance
def get_wire_pointcloud(topic):
    points = rospy.wait_for_message(topic, PointCloud2)
    return points

# Client to call to get wire model 
def process_point_cloud_client(points):
     rospy.wait_for_service('process_point_cloud')
     try:
         wire_nodes = rospy.ServiceProxy('process_point_cloud', ProcessPointCloud)
         response = wire_nodes(points)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

# Client call to grasp and move wire 
def grasp_wire(robot_,wire_grasp_pose,pull_vec):
     rospy.wait_for_service('grasp_wire_service')
     try:
         grasp_wire_input = rospy.ServiceProxy('grasp_wire_service', GraspWire)
         response = grasp_wire_input(robot_,wire_grasp_pose,pull_vec)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

### Slip detect functions
# Client call to swap camera specification 
def set_cam_spec_service(value : Bool):
     rospy.wait_for_service("/set_cam_spec")
     try:
         set_cam_spec = rospy.ServiceProxy('/set_cam_spec', SetBool)
         response = set_cam_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def check_frame_exists(listener, source : str = "world", target : str = "line_grasp_mounted_cam"):
    """
    Check if a given frame exists
    """
    return listener.waitForTransform(source, target, rospy.Time(), rospy.Duration(1.0)) == None

def check_active_range(cam_time, interval : float = 5.0):
    # Active if difference between current time and camera time is less than the interval
    return (rospy.Time.now().to_sec() - cam_time.data.secs) < interval

def check_connector_noise(listener, parent : str = "world", child : str = "line_grasp_mounted_cam", iterations : int = 5):
    curr = listener.lookupTransform(parent, child, rospy.Time(0))
    for i in range(iterations-1):
        sleep(1)
        if curr == listener.lookupTransform(parent, child, rospy.Time(0)):
            return False # Connector frame was not updated over the duration of n iterations, meaning rear cam has lost sight
    return True # Connector frame updated over the duration of n interations, meaning rear cam has sight

#*** Demo Node Starts Here ***#
if __name__ == "__main__":
    ### OVERALL SEARCH DEMO
    # Search algorithm and clearing obstructing wire obstacles
    # using wire segmentation is demonstrated with the following steps:
    # 0. Initiate unplugging of the connector from mock battery ORU
    # 1. Purposefully induce an engineered slip to require anomaly detection and correction
    # 2. Detect slip using Euclidean distance between the connector and grasping arm's end effector
    # 3. Flag the slip and begin searching for lost connector end with spiral search algorithm
    #    3a. Move searching arm to first search target
    #    3b. Conduct search on search target
    #    3c. If nothing found, move search target and arm and loop
    # 4. Connector falls behind an obstructing 
    # 4. If connector found, transform frame of connector from searching arm to grasping arm and initiate retrieval
    #    Else, begin search elsewhere in environment
    # 5. Return retrieved connector end to originally intending stowing point 

    rospy.init_node('listener', anonymous=True)
    robot_control = RobotControl()
    topic = "/rscamera/depth/points"

    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"
    SEARCHING_ARM   = "left"
    SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"
    arm_ids = ["left","right"]
    
    # Send object grasp robot to pre-grasp, encountering wire
    joint_goal_0 = [54, -11, -8, 105, -56, -22] # prepose start
    joint_goal_1 = [39, 6, -27, 116, -43, -31] # start
    joint_goal_unplug = [22, -6, -12, 129, -29, -40] # unplug
    joint_goal_drop = [37, 19, -61, 154, -87, -62] # drop

    joint_goal3 = [32, -7, -3, -78, 35, 74] # final stowing, maybe tune?
    
    listener = tf.TransformListener()
    active_duration = 5

    # Get segmented pointcloud data
    print("STATUS: Getting PointCloud Instance")
    points = get_wire_pointcloud(topic)

    # Process PC data to get estimated nodes on wire
    print("STATUS: Processing Pc with process_point_cloud_server")
    wire_nodes = process_point_cloud_client(points) # output data structure is a posearray

    # Convert PoseArray into Numpy Array
    wire = np.zeros((3,20))
    raw_data = np.zeros((len(wire_nodes.raw_points.poses),3))

    for i in range(20):
        wire[[0],[i]] = wire_nodes.pose.poses[i].position.x
        wire[[1],[i]] = wire_nodes.pose.poses[i].position.y
        wire[[2],[i]] = wire_nodes.pose.poses[i].position.z


    for i in range(len(wire_nodes.raw_points.poses)):
        raw_data[[i],[0]] = wire_nodes.raw_points.poses[i].position.x
        raw_data[[i],[1]] = wire_nodes.raw_points.poses[i].position.y
        raw_data[[i],[2]] = wire_nodes.raw_points.poses[i].position.z


    # ***** insert wire info here *****
    L = wire_nodes.wire_length # get length from point cloud process
    M = 0.028
    Ks = 13.1579
    Kb = 6.13
    Kd = 5.26
    #Ks = 237.5
    #Kb = 25
    #Kd = 125
    wire_model = WireModel(L,M,Ks,Kb,Kd,wire_nodes.wire_class) # create Wire object

    #*** Get bezier curve ***#
    curve = WireGraspToolbox(raw_data)

    #*** Collision Object ***#
    collisions = Collisions()
    collisions.make_box_env_collision_obj(0.05,0.76,0.6,np.array([0.53,0,0.3])) # Front panel
    collisions.make_box_env_collision_obj(0.457,0.05,0.6,np.array([0.3,-0.382,0.3])) # Side Panel
    collisions.make_box_env_collision_obj(0.05,0.05,0.05,np.array([0.48,0.03,0.27])) # Target Object

    #*** Initialize the WireSim Object ***
    N = 20
    wire_sim_ = WireSim(N,wire_model,target_object, curve, collisions )

    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Sending Wire Config to Simulator ")
    pick, pull , wire_grasping_robot = wire_sim_.simulate(wire)

    print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Robot Actions Returned from Simulator ")
    print("     Pick Point ", pick.flatten())
    print("     Pull Vector ", pull.flatten())
    print("")



    ### START ROUTINE for full demonstration at annual review
    ##  Initialize arms; Sleep, open grippers, and ready pose
    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots")
    for arm in arm_ids: 
        status = robot_control.move_to_target(arm, 'sleep')
        status = robot_control.set_gripper(arm, "open")

    ##  Move grasping arm to wire end and unplug
    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Moving grasping arm to connector end and unplug from battery ORU")
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_0])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_1])
    ## Grip and unplug
    status = robot_control.set_gripper(GRASPING_ARM, "close")
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_unplug])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_1])
    status = robot_control.set_gripper(GRASPING_ARM, "open")
                       
    ### START demo as wire is grasped, earliest wire could slip
    print("STATUS: Begin Scenario C4 with ")
    commands = [ # Commands from A1
        # Send to stowing point, engineering slip
        "status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal_drop])",
    ]
    for command in commands:
        exec(command)
        slip_flag = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
        slip_flag = True
        if (slip_flag): # If slip detected, move arm to retrieve wire
            print("STATUS: Slip detected, initiate retrieval")
            sleep(10) # wait 5-10 real time seconds for slipped wire to settle
            status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
            status = robot_control.set_gripper(GRASPING_ARM, "open")
            
            print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear cam identification successful, send grasping arm for retrieval")

            ### Diego's code to move aside obstructing cable
            if (len(pick) > 0 and len(pull) > 0):
                # Get grasp orientation as quaternion 
                grasp_rotm, grasp_quat = curve.get_wire_grasp_orientation(np.transpose(pick),np.transpose(pull))

                ###### Robot Control Setup ########
                # robot_b = left
                # robot_a = right
                print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots ")

                # Set the Wire Grasp Pose
                wire_grasp_pose = geometry_msgs.msg.Pose()
                wire_grasp_pose.orientation.w = grasp_quat[3]
                wire_grasp_pose.orientation.x = grasp_quat[0]
                wire_grasp_pose.orientation.y = grasp_quat[1]
                wire_grasp_pose.orientation.z = grasp_quat[2]
                wire_grasp_pose.position.x = float(pick[0]) 
                wire_grasp_pose.position.y = float(pick[1])
                wire_grasp_pose.position.z = float(pick[2])

                # Set Pull Vector
                pull_vec = geometry_msgs.msg.Vector3()
                pull_vec.x = float(pull[0])
                pull_vec.y = float(pull[1])
                pull_vec.z = float(pull[2])

                #Task Executor
                if(wire_grasping_robot == "left"):
                    object_grasping_robot = "right"
                else:
                    object_grasping_robot = "left"

                #grasp wire
                status = robot_control.grasp_wire(wire_grasping_robot,wire_grasp_pose,pull_vec)
                status = robot_control.set_gripper(wire_grasping_robot, "close")

            status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
            status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_mounted_cam")
            
            # Restore connector to final pose
            status = robot_control.set_gripper(GRASPING_ARM, "close")
            status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal3])  

    ### END SCENARIO C4

    # rospy.spin()
