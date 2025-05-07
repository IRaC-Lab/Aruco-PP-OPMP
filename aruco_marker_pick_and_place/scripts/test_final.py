#!/usr/bin/env python3
from __future__ import print_function

import rospy

import time
import cv2
import cv2.aruco as aruco
import numpy as np
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_commander.conversions import pose_to_list

from visualization_msgs.msg import Marker

#from dynamixel_workbench_msgs.srv import DynamixelCommand

#publish target pose to rviz
import tf2_ros
import tf.transformations
from tf.transformations import *
import tf
import math

from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R

class ManipulatorPickPlace:
    def __init__(self):
        
        rospy.init_node('manipulator_pick_place')

        


        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.marker_size = 0.05  # Size of the marker in meters
        self.camera_matrix = np.array([[499.7530783571702, 0, 316.587604963782],
                                       [0, 497.1254098843589, 244.4671143837159],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([0.1875667734616699, -0.308355301175031, -0.001445054342809013, -0.003001227280700795, 0])
        #set group name
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.move_group = MoveGroupCommander("arm")
        #set end effector
        self.move_group.set_end_effector_link("end_effector_link")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        resize_img = cv2.resize(cv_image, dsize=(640,480), interpolation=cv2.INTER_AREA)
        
        aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = aruco.detectMarkers(resize_img, aruco_dict, parameters=parameters)
        #ids check

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(ids.size):
                aruco.drawAxis(resize_img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                # Perform manipulator pick and place task based on marker pose
                
                self.publish_marker(rvecs[0][0],tvecs[0][0])

        cv2.imshow('ArUco Marker Detection', resize_img)
        cv2.waitKey(1)

        #self.test_publish_marker()

    #def test_publish_marker(self):
    def publish_marker(self, rvecs, tvecs):
    
        pose_aruco = PoseStamped()
        pose_aruco.header.frame_id = "world"
        
        tf_buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buf)
        
        rate = rospy.Rate(10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # transform
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
        
        A = np.array([ [r1[0][0],r1[0][1],r1[0][2], x1_pos], [r1[1][0],r1[1][1],r1[1][2], y1_pos], [r1[2][0],r1[2][1],r1[2][2], z1_pos] ])
        
        B = np.array([tvecs[2],-tvecs[0],-tvecs[1], 1])
        
        result_np = np.dot(A, B)
        
        print(result_np)
        
        #print(k.pose.orientation)
      
        rate.sleep()

if __name__ == '__main__':
    manipulator = ManipulatorPickPlace()
    rospy.spin()
