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


def tcp_sim():

    # Setup port and socket
    sever_port=('127.0.0.1', 54320)

    try:
        # create an AF_INET, STREAM socket (TCP)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except:
        print('Failed to create a socket. ')
        sys.exit()
    print('Socket Created :)')
    sock.connect(sever_port)

    # Starting client
    print("start a client")
    time.sleep(0.01)
    s="start"
    sock.send(s)
    print(s)

    state_msg = ModelState()
    state_msg.model_name = 'wamv'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.3
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    state_msg.twist.linear.x = 0
    state_msg.twist.linear.y = 0
    state_msg.twist.linear.z = 0
    state_msg.twist.angular.x = 0
    state_msg.twist.angular.y = 0
    state_msg.twist.angular.z = 0
    while 1:
        # Receive data
        buf = sock.recv(1000)
        print("Received data:")
        print(buf)
  
        # Load data
        buf_l = json.loads(buf)
        print("Loaded data:")
        print(buf_l)
  
        #state_msg.pose.position.z = buf_l[0]
        #state_msg.pose.position.y = buf_l[1]
        state_msg.pose.position.x = buf_l[0]
        state_msg.pose.position.y = buf_l[1]

        q = euler_to_quaternion(buf_l[2], 0, buf_l[3])
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        state_msg.twist.linear.x = buf_l[4]
        state_msg.twist.linear.y = buf_l[5]
        state_msg.twist.angular.x = buf_l[6]
        state_msg.twist.angular.z = buf_l[7]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        control_signal = 1
        s = str(control_signal)
        sock.send(s)
        print("***")

    sock.close()

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        tcp_sim()
    except rospy.ROSInterruptException:
        pass
