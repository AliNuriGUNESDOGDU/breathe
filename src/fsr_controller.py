#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to FSRCTL
This module uses PEP-8 as coding standard.
"""

import argparse
import copy
import math
import numpy as np
import sys
import serial
import timeit

import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import moveit_commander
import moveit_msgs.srv
import rosparam
import rospy
import sensor_msgs.msg
import std_msgs.msg
import tf
import trajectory_msgs.msg

import robot_joint_space as pp

class FSRCTL(object):
    """This class implements breathing main class
    implements an action client
    send_default():
    method3():
    """
    def __init__(self, args):
        rospy.init_node('serial-arduino', anonymous=True)

        self.file = args.file

        self.ser = serial.Serial(args.port, args.baudrate)
        self.fsr_pub = None
        self.next = False
        self.start_publisher()
        self.cyclic_message()

    
    def start_publisher(self):
        self.fsr_pub = rospy.Publisher(
            'next_sequence', std_msgs.msg.Bool, queue_size=1)

    def cyclic_message(self):
        iter = 0
        while not rospy.is_shutdown():
            iter += 1
            # Sample down the messaging frequency by factor of 25
            if iter > 25:
                iter = 0
                self.fsr_pub.publish(self.next)
            self.serial_step()      

    def serial_step(self):
        start = timeit.default_timer()
        fsr_val = self.ser.readline()
        print(fsr_val)
        stop = timeit.default_timer()
        print(stop-start)

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-p", "--port", type=str, default="/dev/ttyACM0",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-b", "--baudrate", type=int, default="9600",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-f", "--file", type=str, default="subject",
                            help="Value between 0.0 and 0.2")
        args = parser.parse_args()
        print (args.port, args.file)

        fsr_ctl = FSRCTL(args)

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
