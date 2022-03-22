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
        rospy.init_node('fsr_read', anonymous=True)

        self.file = args.file

        self.ser = serial.Serial(args.port, args.baudrate)
        self.fsr_val = std_msgs.msg.UInt16MultiArray()
        self.fsr_pub = None
        self.fsr = False
        self.fsr_readings = []
        self.treshold = 1200
        self.start_publisher()
        print("reading fsr ...")
        self.cyclic_message()
    
    def start_publisher(self):
        self.fsr_pub = rospy.Publisher(
            'fsr_sequence', std_msgs.msg.UInt16MultiArray, queue_size=1)

    def cyclic_message(self):
        self.ser.reset_input_buffer()
        while not rospy.is_shutdown():
            self.fsr_pub.publish(self.fsr_val)
            self.serial_step()   

    def serial_step(self):
        start = timeit.default_timer()
        fsr_val_str = self.ser.readline()
        fsr_val_str_lst = list(fsr_val_str[0:len(fsr_val_str)-2].split(","))
        fsr_lst = []
        try:
            fsr_lst = [int(element) for element in fsr_val_str_lst]
        except:
            print("\n\n\n\n\n\nError occured\n\n\n\n\n\n\n")
            pass
        self.fsr_val = std_msgs.msg.UInt16MultiArray(data = fsr_lst)
        running_mean_fsr = sum(fsr_lst)
        if running_mean_fsr > self.treshold:
            self.fsr = True
        else:
            self.fsr = False
        stop = timeit.default_timer()
        #print(stop-start)

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-p", "--port", type=str, default="/dev/ttyACM0",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-b", "--baudrate", type=int, default="115200",
                            help="Value between 0.0 and 0.2")
        parser.add_argument("-f", "--file", type=str, default="subject",
                            help="Value between 0.0 and 0.2")
        args = parser.parse_args()
        print (args.port, args.file)

        fsr_ctl = FSRCTL(args)

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
