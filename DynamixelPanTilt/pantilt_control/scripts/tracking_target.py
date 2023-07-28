#!/usr/bin/env python3

import rospy
import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose2D
from types import SimpleNamespace
import time

now_pos = [0.0, 0.0]
pre = SimpleNamespace(x=0.0, y=0.0, sum_x=0, sum_y=0)

def callback_target(msg):
    x = 0.8 * pre.x + 0.2 * max(-1, min(msg.x, 1)) # smoothing
    y = 0.8 * pre.y + 0.2 * max(-1, min(msg.y, 1)) # smoothing
    dx = x - pre.x
    dy = y - pre.y
    sum_x = 0.9*pre.sum_x + x
    sum_y = 0.9*pre.sum_y + y
    pre.x = x
    pre.y = y
    pre.sum_x = sum_x
    pre.sum_y = sum_y

    #pid control
    next_pos_x = now_pos[0] - (p_gain * x + d_gain * dx + i_gain * sum_x)
    next_pos_y = now_pos[1] - (p_gain * y + d_gain * dy + i_gain * sum_y)

    point = JointTrajectoryPoint()
    point.positions = (
        max( pan_joint_limit['min'], min(next_pos_x,  pan_joint_limit['max'])),
        max(tilt_joint_limit['min'], min(next_pos_y, tilt_joint_limit['max']))
    )
    point.time_from_start = rospy.Duration.from_sec(step_time)
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = [pan_joint_name, tilt_joint_name]

    joint_trajectory.points.append(point)
    pub_pantilt.publish(joint_trajectory)

def callback_joint(msg):
    now_pos[0] = msg.desired.positions[0]
    now_pos[1] = msg.desired.positions[1]

if __name__=="__main__":
    rospy.init_node('tracking_target_node')

    pan_joint_name  = rospy.get_param('~pan_joint_name', 'base_pantilt_joint')
    tilt_joint_name = rospy.get_param('~tilt_joint_name', 'base_realsense_joint')
    pan_joint_limit = rospy.get_param('~pan_joint_limit', {'min':-math.pi, 'max':math.pi})
    tilt_joint_limit = rospy.get_param('~tilt_joint_limit', {'min':-math.pi, 'max':math.pi})
    p_gain = rospy.get_param('~p_gain', 1.0)
    d_gain = rospy.get_param('~d_gain', 0.05)
    i_gain = rospy.get_param('~i_gain', 0.01)
    step_time = rospy.get_param('~step_time', 0.5)
    
    rospy.Subscriber('/target_position/ratio', Pose2D, callback_target)
    rospy.Subscriber('/pantilt_controller/state', JointTrajectoryControllerState, callback_joint)
    pub_pantilt = rospy.Publisher('/pantilt_controller/command', JointTrajectory, queue_size=10)
    rospy.spin()
    cv2.destroyAllWindows()
