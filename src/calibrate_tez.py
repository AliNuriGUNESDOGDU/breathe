#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to breathe
This module uses PEP-8 as coding standard.
"""

import argparse
from ast import ClassDef
import copy
import math
import numpy as np
import os
import sys

import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import moveit_commander
import moveit_msgs.srv
import rosparam
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg

import gripper_joint_space_vida as pp

class CalibratePositions(object):
    """This class implements breathing main class
    implements an action client
    send_default():
    method3():
    """

    def __init__(self):
        self.curr_state = sensor_msgs.msg.JointState()
        self.subs_node = None 
        self.params = {
            "wait":None, 
            "pick1":None,
            "ready_pick1":None,
            "pick2":None,
            "ready_pick2":None,
            "pick3":None,
            "ready_pick3":None,
            "pick4":None,
            "ready_pick4":None,
            "release":None,
            "ready_release":None,
            "pass":None,
            "show":None,}  
        # self.params.wait_pos = None
        # self.params.pick_pos = None
        # self.params.bef_pick_pos = None
        # self.params.go_pick_pos = None
        # self.params.after_pick_pos = None
        # self.params.release_pos = None
        # self.params.pass_pos = None
        # self.params.show_pos = None
        self.file_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config/test.yaml')

    def callback_subs(self,data):
        self.curr_state = data
    def start_publisher(self):
        self.subs_node = rospy.Subscriber(
            "/joint_states", 
            sensor_msgs.msg.JointState,
            self.callback_subs)
    def calibrate_positions(self):
        raw_input("Hello")
        raw_input("wait position")
        self.params.update({"wait": self.curr_state.position})
        raw_input("pass position")
        self.params.update({"pass": self.curr_state.position})
        raw_input("show position")
        self.params.update({"show": self.curr_state.position})
        raw_input("pick1 position")
        self.params.update({"pick1": self.curr_state.position})
        raw_input("ready_pick1 position")
        self.params.update({"ready_pick1": self.curr_state.position})
        raw_input("pick2 position")
        self.params.update({"pick2": self.curr_state.position})
        raw_input("ready_pick2 position")
        self.params.update({"ready_pick2": self.curr_state.position})
        raw_input("pick3 position")
        self.params.update({"pick3": self.curr_state.position})
        raw_input("ready_pick3 position")
        self.params.update({"ready_pick3": self.curr_state.position})
        raw_input("pick4 position")
        self.params.update({"pick4": self.curr_state.position})
        raw_input("ready_pick4 position")
        self.params.update({"ready_pick4": self.curr_state.position})
        raw_input("release position")
        self.params.update({"release": self.curr_state.position})
        raw_input("ready_release position")
        self.params.update({"ready_release": self.curr_state.position})
        rosparam.upload_params("calib_params", self.params)
        rosparam.dump_params(self.file_path, "calib_params")

def callback(data):    
    curr_states = sensor_msgs.msg.JointState
    curr_states = data
    print(curr_states)



if __name__ == '__main__':
    try:        
        rospy.init_node('test',anonymous=True)
        cp = CalibratePositions()
        cp.start_publisher()
        cp.calibrate_positions()
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
