#!/usr/bin/env python

import sys
import rospy
from xarm_planner.srv import *
from geometry_msgs.msg import Pose

"""
Python wrapper for ROS service client functions. 
This can requesting cartesian plans, joint plans, and execution of planned motions for a robotic arm controlled by the "xarm_planner" node.

Functions:
- request_srv_cartesian_plan(x, y, z, qw, qx, qy, qz): Requests a cartesian plan for a robotic arm's end effector to move to a desired position and orientation in 3D space by communicating with the "xarm_straight_plan" ROS service.
- request_srv_joint_plan(input_joints): Requests a joint plan for a robotic arm's end effector to move to a desired configuration in joint space by communicating with the "xarm_joint_plan" ROS service.
- request_srv_execute_plan(request_boolean): Requests the execution of a previously planned motion by communicating with the "xarm_exec_plan" ROS service.


Author:
- Mark Lee (MoonRobotics@cmu.edu)

Date:
- Apr 2023.
"""


class Srv2Python():
    def __init__(self):
        print(f" ---- creating Service2Python Wrapper ----")
        # self.stuff = 0


    def request_srv_cartesian_line_plan(self, x,y,z, qw,qx,qy,qz):
        """
        Requests a cartesian plan for a robotic arm's end effector to move to a desired position and orientation in 3D space by communicating with the "xarm_straight_plan" ROS service.
        
        Args:
        - x (float): The desired x-coordinate of the target position.
        - y (float): The desired y-coordinate of the target position.
        - z (float): The desired z-coordinate of the target position.
        - qw (float): The w component of the desired quaternion orientation of the target pose.
        - qx (float): The x component of the desired quaternion orientation of the target pose.
        - qy (float): The y component of the desired quaternion orientation of the target pose.
        - qz (float): The z component of the desired quaternion orientation of the target pose.
        
        Returns:
        - success (bool): A boolean value indicating whether the service call to the xarm planner was successful or not.
        
        Raises:
        - rospy.ServiceException: If the service call to the xarm planner fails.
        """

        # hang here until xarm_planner node publishes this srv
        rospy.wait_for_service('xarm_straight_plan')

        try:
            #geometry msg Pose srv
            requesting_geometry_pose = rospy.ServiceProxy('xarm_straight_plan', single_straight_plan)
            
            #init datatype
            target = Pose() 

            #update with input param
            target.position.x = x
            target.position.y = y
            target.position.z = z

            target.orientation.x = qx
            target.orientation.x = qy
            target.orientation.x = qz
            target.orientation.w = qw

            # srv request to xarm planner
            reponse = requesting_geometry_pose(target)

            return reponse.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def request_srv_cartesian_pose_plan(self, x,y,z, qw,qx,qy,qz):


            # hang here until xarm_planner node publishes this srv
            rospy.wait_for_service('xarm_pose_plan')

            try:
                #geometry msg Pose srv
                requesting_geometry_pose = rospy.ServiceProxy('xarm_pose_plan', pose_plan)
                
                #init datatype
                target = Pose() 

                #update with input param
                target.position.x = x
                target.position.y = y
                target.position.z = z

                target.orientation.x = qx
                target.orientation.x = qy
                target.orientation.x = qz
                target.orientation.w = qw

                # srv request to xarm planner
                reponse = requesting_geometry_pose(target)

                return reponse.success
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            

    
    def request_srv_joint_plan(self, input_joints):
        """
        Requests a joint plan for a robotic arm's end effector to move to a desired configuration in joint space by communicating with the "xarm_joint_plan" ROS service.

        Args:
        - input_joints (List[float]): A list of target joint positions in radians.

        Returns:
        - success (bool): A boolean value indicating whether the service call to the xarm planner was successful or not.

        Raises:
        - rospy.ServiceException: If the service call to the xarm planner fails.
        """
        # hang here until xarm_planner node publishes this srv
        rospy.wait_for_service('xarm_joint_plan')

        try:
            # list of target joint positions srv (in radians)
            requesting_joints = rospy.ServiceProxy('xarm_joint_plan', joint_plan) 

            #update with input param
            target = input_joints
            # srv request to xarm planner
            reponse = requesting_joints(target)

            return reponse.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    
    def request_srv_execute_plan(self,request_boolean):
        """
        Requests the execution of a previously planned motion by communicating with the "xarm_exec_plan" ROS service.

        Args:
        - request_boolean (bool): A boolean value indicating whether to execute the previously planned motion.

        Returns:
        - success (bool): A boolean value indicating whether the service call to the xarm planner was successful or not.

        Raises:
        - rospy.ServiceException: If the service call to the xarm planner fails.
        """
        # hang here until xarm_planner node publishes this srv
        rospy.wait_for_service('xarm_exec_plan')

        try:
            # boolean to execute
            requesting_execution = rospy.ServiceProxy('xarm_exec_plan', exec_plan) 

            #update with input param
            target = request_boolean
            # srv request to xarm planner
            reponse = requesting_execution(target)

            return reponse.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
if __name__ == "__main__":
    print(" ================ testing main of srv2python wrapper ============ ")
