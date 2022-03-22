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
    """This class implements fsr read class
    send_default():
    method3():
    """
    def __init__(self):
        self.fsr_sub = None
        self.change = False
        self.fsr = False
        self.fsr_old = False
        self.next = False
        self.fsr_readings = []
        self.treshold = 1200
        self.counter = 0
        self.start_subscriber()
    
    def start_subscriber(self):
        self.fsr_sub = rospy.Subscriber(
            "fsr_sequence", std_msgs.msg.Bool, self.subs_callback)

    def subs_callback(self,data):
        self.fsr = data.data
        if ((self.fsr != self.fsr_old) and (not self.change)):
            self.change = True
            self.fsr_old = self.fsr
            if self.fsr:
                self.next = True
            else:
                self.next = False

        if self.change:
            if self.counter < 4:
                self.counter += 1
            else:
                self.counter = 0
                self.change = False

    def test(self):
        while not rospy.is_shutdown():
            print("sensor",self.fsr,"bounced",self.next)
            if self.next:
                while self.next:
                    print("waiting")
                    rospy.sleep(0.1)
                print("next")
            rospy.sleep(0.1)
            pass

    def wait(self):
        while not rospy.is_shutdown():
            print("sensor",self.fsr,"bounced",self.next)
            if self.next:
                while self.next:
                    print("waiting")
                    rospy.sleep(0.1)
                print("next")
                return True
            rospy.sleep(0.1)
            pass
            
        
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
        rospy.init_node('fsr_test', anonymous=True)
        fsr_ctl_test = FSRCTL()
        print("Testing started ...")
        fsr_ctl_test.test()

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
