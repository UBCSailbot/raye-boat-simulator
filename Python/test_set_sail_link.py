#!/usr/bin/env python    
# -*- coding: utf-8 -*

import socket
import time
import json

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelConfiguration

import numpy as np
import os
from subprocess import call


def main():

    # bashCommand = "source /opt/ros/melodic/setup.zsh && source /home/tylerlum/vrx_ws/devel/setup.zsh && rosservice call /gazebo/set_model_configuration"
    bashCommand = "rosservice call /gazebo/set_model_configuration"
    parameters = ("model_name: 'wamv'\n" +
                   "urdf_param_name: 'robot_description'\n" +
                   "joint_names:\n" +
                   "- 'sail_joint'\n" +
                   "joint_positions:\n" +
                   "- 0.5\"")
    import subprocess
    command = bashCommand.split()
    command.append(parameters)
    # res = subprocess.check_output(["rostopic", "list"])
    res = subprocess.check_output(command)
    for line in res.splitlines():
        print(line)
    return

    import subprocess, os
    my_env = os.environ.copy()
    my_env["PATH"] = "/usr/sbin:/sbin:" + my_env["PATH"]
    print(bashCommand + ' ' + parameters)
    subprocess.Popen(bashCommand + ' ' + parameters, env=my_env)

    bash = bashCommand.split()
    bash.append(parameters)
    import subprocess
    call(bashCommand + ' ' + parameters, shell=True)
    # process = subprocess.Popen(bash, stdout=subprocess.PIPE)
    process = call(bash, shell=True, executable="/bin/bash")
    output, error = process.communicate()

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
        main()
    except rospy.ROSInterruptException:
        pass
