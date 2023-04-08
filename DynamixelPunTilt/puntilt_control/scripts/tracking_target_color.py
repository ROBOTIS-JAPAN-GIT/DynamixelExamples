#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import time


def callback_img(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 50, 0])
    upper_red = np.array([10, 255, 255])
    red_mask1 = cv2.inRange(hsv_img, lower_red, upper_red)

    lower_red = np.array([160, 50, 0])
    upper_red = np.array([180, 255, 255])
    red_mask2 = cv2.inRange(hsv_img, lower_red, upper_red)

    red_mask = red_mask1 + red_mask2

    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    print(now_pos)
    point = JointTrajectoryPoint()
    point.positions = [0,0]
    point.time_from_start = rospy.Duration.from_sec(1)
    
    if len(contours)!=0:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        print(msg.width/2 - (x+w/2), msg.height/2 - (y+h/2))
        point.positions =(
            now_pos[0] + p_gain * (msg.width /2 - (x+w/2)) / msg.width, 
            now_pos[1] + p_gain * (msg.height/2 - (y+h/2)) / msg.height
        )

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = [pan_joint_name, tilt_joint_name]

    joint_trajectory.points.append(point)
    pub_puntilt.publish(joint_trajectory)

def callback_joint(msg):
    global now_pos
    now_pos = msg.desired.positions


if __name__=="__main__":
    rospy.init_node('tracking_target_color_node')

    pan_joint_name = rospy.get_param('~pan_joint_name', 'base_puntilt_joint')
    tilt_joint_name = rospy.get_param('~tilt_joint_name', 'base_realsense_joint')
    p_gain = rospy.get_param('~p_gain', 1.0)

    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)
    rospy.Subscriber('/puntilt_controller/state', JointTrajectoryControllerState, callback_joint)
    pub_puntilt = rospy.Publisher('/puntilt_controller/command', JointTrajectory, queue_size=10)
    rospy.spin()
    cv2.destroyAllWindows()