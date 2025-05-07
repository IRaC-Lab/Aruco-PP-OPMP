#!/usr/bin/env python3
from __future__ import print_function

import rospy
import sys
import math
import time
import numpy as np
import actionlib

# for OpenCV marker

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# for moveit commader

import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_commander.conversions import pose_to_list

from visualization_msgs.msg import Marker

#publish target pose to rviz

import tf
import tf2_ros
import tf.transformations
from tf.transformations import *
from tf import TransformListener


# for transform

from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from scipy.linalg import inv

# send msg
from std_msgs.msg import String

# gripper control
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class ManipulatorPickPlace:
    
    def __init__(self):
        
        rospy.init_node('manipulator_pick')

        # moveit part
        # set group name
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        self.move_group = MoveGroupCommander("arm")
        
        # set gripper
        
        self.gripper_group = MoveGroupCommander("gripper")
        
        # set end effector
        self.move_group.get_planning_frame()
        self.move_group.set_end_effector_link("end_effector_link")
        
        # gripper 
        self.grip = ['gripper']
        self.gripper_client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client.wait_for_server()        
        rospy.loginfo('...connected.')
        
        self.perform_pick_place_task()
        
    def get_trans(self):
        pose_aruco = PoseStamped()
        pose_aruco.header.frame_id = "world"
            
        tf_buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buf)
            
        rate = rospy.Rate(1000)
        try:   
            #self.trans = tf_buf.lookup_transform("world",'aruco_marker',rospy.Time(0))       
            self.trans = tf_buf.lookup_transform_full(target_frame = 'aruco_marker', target_time = rospy.Time(0), source_frame = 'world', source_time = rospy.Time.now(), fixed_frame = "world", timeout=rospy.Duration(3))
                
            rospy.sleep(1) 
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()     
            
    def perform_pick_place_task(self):

        pose_aruco = PoseStamped()
        pose_aruco.header.frame_id = "world"
        
        tf_buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buf)
        
        rate = rospy.Rate(10)
        
        # measure time start
        start = time.time()
            
        self.get_trans()
        
        #Execute Pick
        
        #print(trans_pose)
        
        # if want to set target named pose, Use move_group.set_named_target("pose name")
               
        # Gripper Open

        # sim
        #gripper_open = [0.007]
        #gripper_close = [-0.007]
        
        # real
        gripper_open = [-0.007]
        gripper_close = [0.0005]


        gripper_traj_open = JointTrajectory()
        gripper_traj_open.joint_names = self.grip
        gripper_traj_open.points.append(JointTrajectoryPoint())
        gripper_traj_open.points[0].positions = gripper_open
        gripper_traj_open.points[0].velocities = [0.0 for i in self.grip]
        gripper_traj_open.points[0].accelerations = [0.0 for i in self.grip]
        gripper_traj_open.points[0].time_from_start = rospy.Duration(3.0)

        gripper_traj_close = JointTrajectory()
        gripper_traj_close.joint_names = self.grip
        gripper_traj_close.points.append(JointTrajectoryPoint())
        gripper_traj_close.points[0].positions = gripper_close
        gripper_traj_close.points[0].velocities = [0.0 for i in self.grip]
        gripper_traj_close.points[0].accelerations = [0.0 for i in self.grip]
        gripper_traj_close.points[0].time_from_start = rospy.Duration(3.0)

        
        gripper_goal = FollowJointTrajectoryGoal()
        
        gripper_goal.trajectory = gripper_traj_open
        gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))
        
        # Perform pick operation        

        print()
        
        # Set Target Pose
        pose_aruco.pose.position.x = round(-self.trans.transform.translation.z +0.00, 4)
        pose_aruco.pose.position.y = round(-self.trans.transform.translation.y, 4)
        pose_aruco.pose.position.z = round(self.trans.transform.translation.x + 0.00, 4) 

        pose_aruco.pose.orientation.x = round(-self.trans.transform.rotation.x, 4)
        pose_aruco.pose.orientation.y = round(-self.trans.transform.rotation.y, 4)
        pose_aruco.pose.orientation.z = round(-self.trans.transform.rotation.z, 4)
        pose_aruco.pose.orientation.w = round(self.trans.transform.rotation.w, 4)
        print(pose_aruco)
        # Move the manipulator to the desired pick pose

        pick_pose = pose_aruco
        
        self.move_group.set_goal_tolerance(0.03)
        self.move_group.set_pose_target(pick_pose)
        self.move_group.go(wait=True)        
        self.move_group.stop()     
        self.move_group.clear_pose_targets()
        print("target")
            
        # measure time end     
        end = time.time()
        print(f"{end - start:.3f} sec")  
        
        print("end effector point: ", self.move_group.get_current_pose())
        
        # check the robot end effetor point
        time.sleep(3)
        
        # 0.170 0.003 0.047
        
        get_place_pose = Pose()

        #get_place_pose.position.x = 0.17  # Set the desired place position
        #get_place_pose.position.y = -0.002246
        #get_place_pose.position.z = 0.057030
        
        #get_place_pose.position.x = 0.175  # Set the desired place position
        #get_place_pose.position.y = -0.002246
        #get_place_pose.position.z = 0.057030
        """
        get_place_pose.position.x = 0.1871  # Set the desired place position
        get_place_pose.position.y = -0.0017
        get_place_pose.position.z = 0.0653
        
        get_place_pose.orientation.x = 0.005423
        get_place_pose.orientation.y = 0.707086
        get_place_pose.orientation.z = -0.005423
        get_place_pose.orientation.w = 0.707086
        
        print(get_place_pose)
        
        self.move_group.set_pose_target(get_place_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        """
        
        
        ### erase """ from
        """
        
        # close gripper
        gripper_goal.trajectory = gripper_traj_close
        gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))    
        
        # Move the manipulator to the desired place pose

        self.move_group.set_named_target("detect")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        place_pose = Pose()

        place_pose.position.x = 0.07011  # Set the desired place position
        place_pose.position.y = 0.11370
        place_pose.position.z = 0.058030
        
        place_pose.orientation.x = -0.358762
        place_pose.orientation.y = 0.586253
        place_pose.orientation.z = 0.379142
        place_pose.orientation.w = 0.619556
        
        self.move_group.set_pose_target(place_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory = gripper_traj_open
        gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))       
        
        place_pose.position.x = 0.07011  # Set the desired place position
        place_pose.position.y = 0.11370
        place_pose.position.z = 0.110030
        
        
        self.move_group.set_pose_target(place_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        """
        # Perform place operation (e.g., open gripper)
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        """
 
	"""
	# erase """ to  
        
        # return to home Pose
        self.move_group.set_named_target("detect2")
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        rospy.sleep(1)  # Wait for stability or any other necessary delay
        
        
if __name__ == '__main__':


        manipulator = ManipulatorPickPlace()          
            
        #rospy.spin()


