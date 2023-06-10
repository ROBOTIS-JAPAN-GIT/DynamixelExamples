#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose2D
from types import SimpleNamespace
import time

now_pos = [0.0, 0.0]
pre = SimpleNamespace(x=0.0, y=0.0, sum_x=0, sum_y=0)

def callback_target(msg):
    x = max(-1, min(msg.x, 1))
    y = max(-1, min(msg.y, 1))
    dx = pre.x - x
    dy = pre.y - y
    sum_x = max(-100, min(0.8*pre.sum_x + x, 100))
    sum_y = max(-100, min(0.8*pre.sum_y + y, 100))
    pre.x = x
    pre.y = y
    pre.sum_x = sum_x
    pre.sum_y = sum_y

    point = JointTrajectoryPoint()
    point.positions = (
        now_pos[0] - (p_gain * x + d_gain * dx + i_gain * sum_x), 
        now_pos[1] - (p_gain * y + d_gain * dy + i_gain * sum_y)
    )
    point.time_from_start = rospy.Duration.from_sec(0.1)
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = [pan_joint_name, tilt_joint_name]

    joint_trajectory.points.append(point)
    pub_puntilt.publish(joint_trajectory)

def callback_joint(msg):
    now_pos[0] = msg.desired.positions[0]
    now_pos[1] = msg.desired.positions[1]

if __name__=="__main__":
    rospy.init_node('tracking_target_node')

    pan_joint_name  = rospy.get_param('~pan_joint_name', 'base_puntilt_joint')
    tilt_joint_name = rospy.get_param('~tilt_joint_name', 'base_realsense_joint')
    p_gain = rospy.get_param('~p_gain', 1.0)
    d_gain = rospy.get_param('~d_gain', 0.05)
    i_gain = rospy.get_param('~i_gain', 0.01)

    rospy.Subscriber('/target_position/ratio', Pose2D, callback_target)
    rospy.Subscriber('/puntilt_controller/state', JointTrajectoryControllerState, callback_joint)
    pub_puntilt = rospy.Publisher('/puntilt_controller/command', JointTrajectory, queue_size=10)
    rospy.spin()
    cv2.destroyAllWindows()
