#!/usr/bin/env python3
from __future__ import print_function

import rospy
import sys
import math
import time
import numpy as np

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

class ManipulatorPickPlace:
    
    def __init__(self):
        
        rospy.init_node('manipulator_pick_place')

        # OpenCV node
        
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/openmv_cam/image_raw', Image, self.image_callback)

        self.marker_size = 0.03  # Size of the marker in meters
        
        # set camera data
        self.camera_matrix = np.array([[1325.245498, 0, 294.386130],
                                       [0, 1324.674694, 271.713120],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([0.225983, 0.595291, 0.004383, 0.010017, 0])

        # moveit part
        # set group name
        moveit_commander.roscpp_initialize(sys.argv)
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        self.move_group = MoveGroupCommander("arm")
        
        # set end effector
        self.move_group.set_end_effector_link("end_effector_link")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # resize input image
        resize_img = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # set ArUco marker
        aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = aruco.detectMarkers(resize_img, aruco_dict, parameters=parameters)
        
        #ids check
        if ids is not None:
            #get TF vectors
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(ids.size):
                aruco.drawAxis(resize_img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                
                self.publish_marker(rvecs[0][0],tvecs[0][0])

        # to check camera image on PC
        
        dir_img = cv2.rotate(resize_img, cv2.ROTATE_90_CLOCKWISE)
        
        cv2.imshow('ArUco Marker Detection', dir_img)
        cv2.waitKey(1)
        
    # to publish target pose to rviz
     
    def publish_marker(self, rvecs, tvecs):   
      
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()   
        rate = rospy.Rate(5)

        # get Camera Pose
        k = self.move_group.get_current_pose("camera_link")
        
        x1_ori = k.pose.orientation.x
        y1_ori = k.pose.orientation.y
        z1_ori = k.pose.orientation.z
        w1_ori = k.pose.orientation.w
        
        x1_pos = k.pose.position.x
        y1_pos = k.pose.position.y
        z1_pos = k.pose.position.z
        
        r = R.from_quat([x1_ori, y1_ori, z1_ori, w1_ori])
        r1 = r.as_dcm()    
        
        # using scipy 
        rot = R.from_rotvec([rvecs[0], rvecs[1], rvecs[2]]) #standard
        r_matrix = rot.as_dcm()
        
        A = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        matrix = np.dot(A,r_matrix)
        
        B = R.from_dcm(matrix)
        
        #get the Z rot degree
        x_val = x1_pos + tvecs[2]
        y_val = -tvecs[0]
        
        deg = math.atan(y_val/x_val)
        
        #cos = math.cos(math.pi * (deg / 180))
        #sin = math.sin(math.pi * (deg / 180))
        
        cos = math.cos(deg)
        sin = math.sin(deg)
                
        z_mat = np.array([[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]])
        y_mat = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        
        rot_grip = np.dot(z_mat, y_mat)
        
        #quat = B.as_quat()
        #quat = [ 0.70489508, -0.70023717, -0.07868303,  0.08123925]
        
        # using scipy inverse
        
        r1_inv = inv(r1)
        
        #r1_inv_r = R.from_dcm(r1_inv)
        #quat_inv = r1_inv_r.as_quat()
        
        # marker in rot
        fin_marker = R.from_dcm(np.dot(rot_grip,r1_inv))
        quat_inv = fin_marker.as_quat()
        
        
        # Publish marker Pose
        # don't change
        aruco_marker_tf = TransformStamped()
        aruco_marker_tf.header.stamp = rospy.Time.now()
        aruco_marker_tf.header.frame_id = "camera_link"
        aruco_marker_tf.child_frame_id = "aruco_marker"
        
        # marker trans      
        aruco_marker_tf.transform.translation.x = tvecs[2]
        aruco_marker_tf.transform.translation.y = -tvecs[0]
        aruco_marker_tf.transform.translation.z = -tvecs[1]
        
        # marker ori      
        aruco_marker_tf.transform.rotation.x = quat_inv[0]
        aruco_marker_tf.transform.rotation.y = quat_inv[1]
        aruco_marker_tf.transform.rotation.z = quat_inv[2]
        aruco_marker_tf.transform.rotation.w = quat_inv[3]        
        
        self.tf_broadcaster.sendTransform(aruco_marker_tf)
        
        print(aruco_marker_tf)
                
        rate.sleep()

if __name__ == '__main__':

    manipulator = ManipulatorPickPlace()
	
    rospy.spin()

