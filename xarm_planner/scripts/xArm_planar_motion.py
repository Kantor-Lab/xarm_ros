#!/usr/bin/env python3

import sys
import rospy
from xarm_planner.srv import *
from geometry_msgs.msg import Pose

import xArmPlanner_srv2python_wrapper as xArm_srv_wrapper

"""
Main script to execute that handles xArm motion for Corn Stalk insertion for CMU-ISU 2023 field test. 

Author:
- Mark Lee (MoonRobotics@cmu.edu)

Date:
- Apr 2023.
"""

HOME_POSE = [0.207, 0.0, 0.112, 0., 0., 0., 1.] 
HOME_POSE_WITH_ROT = [0.207, 0.0, 0.112, 0.706675, 0., 0., 0.70754]
MID_PT_POSE_TO_APPROACH = [0.293, 0.0, 0.67, 0.706675, 0., 0., 0.70754]
APPROACH_POSE = [0.293, 0.05, 0.67, 0.706675, 0., 0., 0.70754]
INSERT_POSE = [0.293, -0.25, 0.67, 0.706675, 0., 0., 0.70754]


def get_desired_pose_from_list(input_list):

    x = input_list[0]
    y = input_list[1]
    z = input_list[2]
    qx = input_list[3]
    qy = input_list[4]
    qz = input_list[5]
    qw = input_list[6]

    return x, y, z, qx, qy, qz, qw

def corn_stalk_motion():
    #default home pose
    x,y,z,qx,qy,qz,qw = get_desired_pose_from_list(HOME_POSE)


    request_states = xArm_srv_wrapper.request_srv_cartesian_pose_plan(x,y,z, qx,qy,qz,qw)
    print(f"requesting plan: GO_HOME_POSE, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: GO_HOME_POSE, Completed True/False?: {request_states}")

    #default home pose w rotation
    x,y,z,qx,qy,qz,qw = get_desired_pose_from_list(HOME_POSE_WITH_ROT)

    
    request_states = xArm_srv_wrapper.request_srv_cartesian_pose_plan(x,y,z, qx,qy,qz,qw)
    print(f"requesting plan: HOME_POSE_WITH_ROT, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: HOME_POSE_WITH_ROT, Completed True/False?: {request_states}")

    #Mid pt to approach pose for higher planning success rate
    x,y,z,qx,qy,qz,qw = get_desired_pose_from_list(MID_PT_POSE_TO_APPROACH)

    request_states = xArm_srv_wrapper.request_srv_cartesian_pose_plan(x,y,z, qx,qy,qz,qw)
    print(f"requesting plan: MID_PT_POSE_TO_APPROACH, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: MID_PT_POSE_TO_APPROACH, Completed True/False?: {request_states}")

    #Approach pose before inserting sensor into corn stalk
    x,y,z,qx,qy,qz,qw = get_desired_pose_from_list(APPROACH_POSE)
    

    request_states = xArm_srv_wrapper.request_srv_cartesian_pose_plan(x,y,z, qx,qy,qz,qw)
    print(f"requesting plan: APPROACH_CARTESIAN, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: APPROACH_CARTESIAN, Completed True/False?: {request_states}")

    #pose for inserting sensor into corn stalk
    x,y,z,qx,qy,qz,qw = get_desired_pose_from_list(INSERT_POSE)
   

    request_states = xArm_srv_wrapper.request_srv_cartesian_pose_plan(x,y,z, qx,qy,qz,qw)
    print(f"requesting plan: INSERT_CARTESIAN, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: INSERT_CARTESIAN, Completed True/False?: {request_states}")

if __name__ == "__main__":
    print(" ================ start ============ ")

    #obtained from manually playing with MoveIt and then reading joint postions
    # STOW_JOINT_ANGLES = [-0.005691000155291448, -1.5617350979917042, -3.817223679725146e-05, -0.0002003826846523893, -0.00011472079979313321, 3.616551335650087e-05]
    # APPROACH_JOINT_ANGLES = [-0.02991102667951928, -0.311088608000313, -1.189797271512119, 1.6267551770805806, 1.47070589650495, -0.0957004890775206]

    #create wrapper object
    xArm_srv_wrapper = xArm_srv_wrapper.Srv2Python()

    #execute scripted motion for corn stalk insertion
    corn_stalk_motion()
    print(" ================ completed script ============ ")


    


