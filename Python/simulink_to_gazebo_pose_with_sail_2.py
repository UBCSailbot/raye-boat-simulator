#!/usr/bin/env python    
# -*- coding: utf-8 -*

import socket
import time
import json
import math

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelConfiguration
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

    boat_state_msg = ModelState()
    boat_state_msg.model_name = 'wamv'

    model_name = 'wamv'
    urdf_param_name = 'robot_description'
    joint_names = ['sail_joint']

    while 1:
        # Receive data
        buf = sock.recv(1000)
        print("Received data:")
        print(len(buf))
        i = buf.index('[')
        buf = buf[buf.index('['):]
        buf = buf[:buf.index(']')] + ']'
        print(len(buf))
        print(buf)
        print("***")
  
        # Load data
        buf_l = json.loads(buf)
        print("Loaded data:")
        print(buf_l)
        print("---")

        # Send gazebo control msg
        boat_state_msg = get_updated_boat_state_msg(boat_state_msg, buf_l)

        print("moving boat in right away")
        time.sleep(0.05)
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_boat_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp_1 = set_boat_state( boat_state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        joint_positions = [buf_l[8]]
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            set_boat_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            resp_2 = set_boat_configuration(model_name, urdf_param_name, joint_names, joint_positions)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    sock.close()

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def get_updated_boat_state_msg(boat_msg, data):
    boat_msg.pose.position.x = data[0]
    boat_msg.pose.position.y = data[1]
    boat_msg.pose.position.z = 0

    q = euler_to_quaternion(data[2], 0, data[3])
    boat_msg.pose.orientation.x = q[0]
    boat_msg.pose.orientation.y = q[1]
    boat_msg.pose.orientation.z = q[2]
    boat_msg.pose.orientation.w = q[3]

    boat_msg.twist.linear.x = data[4]
    boat_msg.twist.linear.y = data[5]
    boat_msg.twist.angular.x = data[6]
    boat_msg.twist.angular.z = data[7]
    return boat_msg

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        tcp_sim()
    except rospy.ROSInterruptException:
        pass
