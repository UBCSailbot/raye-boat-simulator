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

######################### MAIN METHOD ######################### 

def main():

    ## Create socket and start
    sock = create_socket()
    send_start_command(sock)

    ## Setup for SetModelState (set model pose)
    boat_state_msg = ModelState()
    boat_state_msg.model_name = 'wamv'

    ## Setup for SetModelState (set wind arrow pose)
    wind_state_msg = ModelState()
    wind_state_msg.model_name = 'post_0'

    ## Setup for SetModelConfiguration (set joint pose)
    model_name = 'wamv'
    urdf_param_name = 'robot_description'
    joint_names = ['sail_joint', 'rudder_joint']

    while True:
        # Get data
        buf = receive_data(sock)
        loaded_buf = load_data(buf)

        # Store data
        model_and_joint_pose = ModelAndJointPose(loaded_buf)
        wind = Wind(loaded_buf)

        # Setup msgs to send to Gazebo
        boat_state_msg = get_updated_boat_state_msg(boat_state_msg, model_and_joint_pose)
        joint_positions = get_updated_joint_positions(model_and_joint_pose)
        wind_state_msg = get_updated_wind_state_msg(wind_state_msg, model_and_joint_pose, wind)

        # Set model pose and joint pose
        set_model_pose(boat_state_msg)
        set_model_pose(wind_state_msg)
        set_joint_pose(model_name, urdf_param_name, joint_names, joint_positions)

    sock.close()

######################### HELPER METHODS ######################### 
def create_socket():
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

    return sock

def send_start_command(sock):
    # Send start command
    s="start"
    sock.send(s)
    print(s)

def receive_data(sock):
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
    return buf

def load_data(buf):
    # Load data
    loaded_buf = json.loads(buf)
    print("Loaded data:")
    print(loaded_buf)
    print("---")
    return loaded_buf

def set_model_pose(boat_state_msg):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_boat_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_boat_state(boat_state_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def set_joint_pose(model_name, urdf_param_name, joint_names, joint_positions):
    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        set_boat_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        resp = set_boat_configuration(model_name, urdf_param_name, joint_names, joint_positions)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def get_updated_boat_state_msg(boat_msg, model_and_joint_pose):
    boat_msg.pose.position.x = model_and_joint_pose.x 
    boat_msg.pose.position.y = model_and_joint_pose.y 

    q = euler_to_quaternion(model_and_joint_pose.phi, 0, model_and_joint_pose.psi)
    boat_msg.pose.orientation.x = q.x 
    boat_msg.pose.orientation.y = q.y 
    boat_msg.pose.orientation.z = q.z 
    boat_msg.pose.orientation.w = q.w 

    return boat_msg

def get_updated_wind_state_msg(wind_msg, model_and_joint_pose, wind):
    height_above_boat = 5
    wind_msg.pose.position.x = model_and_joint_pose.x 
    wind_msg.pose.position.y = model_and_joint_pose.y 
    wind_msg.pose.position.z = height_above_boat

    q = euler_to_quaternion(0, 0, wind.alpha_tw)
    wind_msg.pose.orientation.x = q.x 
    wind_msg.pose.orientation.y = q.y 
    wind_msg.pose.orientation.z = q.z 
    wind_msg.pose.orientation.w = q.w 

    return wind_msg

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(qx, qy, qz, qw)

def get_updated_joint_positions(model_and_joint_pose):
    return [model_and_joint_pose.sangle, model_and_joint_pose.rangle]

######################### DATATYPES ######################### 
# Note: need to change coordinate frame (Gazebo XYZ=NWU, Simulink XYZ=NED) (N = North, W = West, E = East, U = Up, D = Down)
class ModelAndJointPose:
    def __init__(self, loaded_buf):
        self.x = loaded_buf[0]
        self.y = -loaded_buf[1]
        self.phi = loaded_buf[2]
        self.psi = -loaded_buf[3]
        self.sangle = -loaded_buf[4]
        self.rangle = -loaded_buf[5]

class Wind:
    def __init__(self, loaded_buf):
        self.v_tw = loaded_buf[6]
        self.alpha_tw = -loaded_buf[7]

class Quaternion:
    def __init__(self, qx, qy, qz, qw):
        self.x = qx
        self.y = qy
        self.z = qz
        self.w = qw

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
