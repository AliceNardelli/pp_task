#!/usr/bin/python3.8.10
import enum
import math
import time

import rospy
from std_msgs.msg import Int8, String

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import argparse
import os


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_

def __build_arm_pose(x, y, z, o_x, o_y, o_z):
    pose_goal = kinova_msgs.msg.ArmPoseGoal()
    pose_goal.pose.header = std_msgs.msg.Header(frame_id="j2s7s300_link_base")
    pose_goal.pose.pose.position = geometry_msgs.msg.Point(
        x=x,
        y=y,
        z=z
    )
    orientation_q = EulerXYZ2Quaternion(list(map(math.radians, [o_x, o_y, o_z])))
    pose_goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation_q[0],
        y=orientation_q[1],
        z=orientation_q[2],
        w=orientation_q[3]
    )
    return pose_goal


def move1(pose):
        # Init the robot client
        robot_client = actionlib.SimpleActionClient('j2s7s300_driver/pose_action/tool_pose', kinova_msgs.msg.ArmPoseAction)
        robot_client.wait_for_server()
        rospy.loginfo("Waiting for arm driver")
        goal1=__build_arm_pose(pose[0],pose[1], pose[2],pose[3],pose[4], pose[5])
        robot_client.send_goal(goal1)
        if robot_client.wait_for_result(rospy.Duration(60)):
            return robot_client.get_result()
        else:
            rospy.loginfo('        the action timed-out')
            robot_client.cancel_all_goals()
            return None

        



if __name__ == '__main__':

    try:
        rospy.init_node('kinova_controller', anonymous=False)
        home=[0.7, 0.0, 0.1, 70, 50, 90]
        pp=[0.0, -0.75, 0.4, 70, 0, 90]
        for i in range(2):
            move1(home)
            move1(pp)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
