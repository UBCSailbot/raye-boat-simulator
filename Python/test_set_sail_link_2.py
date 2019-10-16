#!/usr/bin/env python    
# -*- coding: utf-8 -*

import socket
import time
import json

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np
import copy

def main():

    state_msg_1 = ModelState()
    state_msg_1.model_name = 'wamv'

    state_msg_2 = ModelState()
    state_msg_2.model_name = 'post'

    buf_l = [5, 0, 0, 0, 0, 0, 0, 0, 0]
    #buf_l_2 = [5, 0, 1.79, 0, -2, 0, 0, 0, 0]
    buf_l_2 = copy.copy(buf_l)
    buf_l_2[2] += 1.79
    while 1:
        buf_l_2[4] += 0.1

        # Send gazebo control msg
        state_msg_1 = get_updated_model_state_msg(state_msg_1, buf_l)
        state_msg_2 = get_updated_model_state_msg(state_msg_2, buf_l_2)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg_1 )
            set_state_2 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp_2 = set_state( state_msg_2 )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def get_updated_model_state_msg(msg, data):
    msg.pose.position.x = data[0]
    msg.pose.position.y = data[1]
    msg.pose.position.z = data[2]

    q = euler_to_quaternion(data[3], 0, data[4])
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    msg.twist.linear.x = data[5]
    msg.twist.linear.y = data[6]
    msg.twist.angular.x = data[7]
    msg.twist.angular.z = data[8]
    return msg

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
