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
    server_port=('127.0.0.1', 54320)

    try:
        # create an AF_INET, STREAM socket (TCP)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except:
        print('Failed to create a socket. ')
        sys.exit()

    print('Socket Created :)')
    sock.connect(server_port)
    print("start a client")
    time.sleep(0.01)

    # Send start command
    s="start"
    sock.send(s)
    print(s)

    state_msg = ModelState()
    state_msg.model_name = 'wamv'

    while 1:
        # Receive data
        buf = sock.recv(1000)
        print("Received data:")
        print(buf)
  
        # Load data
        buf_l = json.loads(buf)
        print("Loaded data:")
        print(buf_l)

        # Send gazebo control msg
        state_msg = get_updated_model_state_msg(state_msg, buf_l)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    sock.close()

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def get_updated_model_state_msg(msg, data):
    msg.pose.position.x = data[0]
    msg.pose.position.y = data[1]

    q = euler_to_quaternion(data[2], 0, data[3])
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    msg.twist.linear.x = data[4]
    msg.twist.linear.y = data[5]
    msg.twist.angular.x = data[6]
    msg.twist.angular.z = data[7]
    return msg

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        tcp_sim()
    except rospy.ROSInterruptException:
        pass
