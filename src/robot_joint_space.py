#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to pick and place
This module uses PEP-8 as coding standard.
"""

import argparse
import copy
import math
import numpy as np
import os
import sys

import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import moveit_commander
import rosparam
import rospy
import tf
import trajectory_msgs.msg

from std_msgs.msg import String


class PickPlace(object):
    """This class implements breathing main class
    implements an action client
    send_default():
    method3():
    """

    def __init__(self):
        
        # Upload parameter server
        file_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config/test.yaml')
        paramlist = rosparam.load_file(file_path,"calib_params")
        for params, ns in paramlist:
            rosparam.upload_params(ns, params)
        # start initial parameters
        self.gripper_pub = None
        self.client = None
        self.goal_j = None


        #rospy.init_node('pick_place', anonymous=True)
        # Start outputs
        self.start_client()
        self.start_publisher()

    def start_publisher(self):
        self.gripper_pub = rospy.Publisher(
            'gripper_control', String, queue_size=1)

    def start_client(self):
        """Method to start action client
        """
        JOINT_NAMES = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint']
        # Check whether simulation or not
        if rospy.has_param('arm_controller'):
            namespace_ = '/arm_controller/follow_joint_trajectory'
        else:
            namespace_ = '/follow_joint_trajectory'
        # Create an action client
        self.client = actionlib.SimpleActionClient(
            namespace_,  # namespace of the action topics
            control_msgs.msg.FollowJointTrajectoryAction  # action type
        )
        print("Waiting for server...")
        self.client.wait_for_server()
        print("Connected to server")
        self.goal_j = control_msgs.msg.FollowJointTrajectoryGoal()
        self.goal_j.trajectory = trajectory_msgs.msg.JointTrajectory()
        self.goal_j.trajectory.joint_names = JOINT_NAMES

    def state_machine(self):
        """State machine of pick place and wait function
        GO_WAIT->WAIT->GO_PICK->PICK->GO_SHOW->SHOW->GO_RELEASE->RELEASE->END
        """
        # Parameters of state machine
        # # Joint states of the pick and place action
        # q_wait = [1.83,1.01,-1.92,-1.18,-2.04,-0.84]
        # q_go_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_show = [1.03,0.83,-1.23,-3.00,-2.10,-1.68]
        # q_release = [-0.37,0.67,-1.21,-1.1,-1.61,-0.85]
        q_wait = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -1.3215044180499476, -1.4791424910174769, -1.658243481312887]
        q_before_pick = [2.19,0.57,-1.07,-1.07,-1.57,-1.04]
        q_go_pick = [2.19,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_pick = q_before_pick
        q_show = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -0.67, -1.4791424910174769, -1.658243481312887]
        q_before_release = [1.12,0.57,-1.07,-1.07,-1.57,-1.04]
        q_release = [1.12,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_release = q_before_release
        # Times
        t_go_wait = 0.8
        t_wait = 0.4
        t_before_pick = 3.4
        t_go_pick = 1.0
        t_gripper_open = 0.3
        t_pick = 0.5
        t_gripper_close = 0.2
        t_after_pick = 1.0
        t_go_show = 2.4
        t_show = 0.1
        t_before_release = 2.8
        t_go_release = 1.0
        t_release = 1.0
        t_after_release = 1.0
        t_go_end = 4.4
        # Other Params
        rate = 20 # Hz
        t_passed = 0.0        
        # States
        GO_WAIT         = False
        WAIT            = False
        GO_PICK         = False
        PICK            = False
        GO_SHOW         = False
        SHOW            = False
        GO_RELEASE      = False
        RELEASE         = False
        END             = False
        IN_GO_WAIT      = True
        IN_WAIT         = False
        IN_BEFORE_PICK  = False
        BEFORE_PICK     = False
        IN_GO_PICK      = False
        IN_PICK         = False
        IN_AFTER_PICK   = False
        AFTER_PICK      = False
        IN_GO_SHOW      = False
        IN_SHOW         = False
        IN_BEFORE_RELEASE= False
        BEFORE_RELEASE   = False
        IN_GO_RELEASE   = False
        IN_RELEASE      = False
        IN_AFTER_RELEASE= False
        AFTER_RELEASE   = False
        IN_END          = False

        # State Machine
        sample = rospy.Rate(rate)
        while not rospy.is_shutdown():            
            sample.sleep()
            if IN_GO_WAIT:
                self.go(q_wait,t_go_wait)
                IN_GO_WAIT = False
                GO_WAIT = True
                print("IN_GO_WAIT")
                pass
            elif GO_WAIT:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_WAIT = False
                    IN_WAIT = True
                    pass
                pass
            elif IN_WAIT:
                t_passed = 0.0
                IN_WAIT = False
                WAIT = True
                print("IN_WAIT")
                pass
            if WAIT:
                if t_passed >= t_wait:
                    WAIT = False
                    IN_BEFORE_PICK = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_BEFORE_PICK:
                self.go(q_before_pick,t_before_pick)
                IN_BEFORE_PICK = False
                BEFORE_PICK = True
                print("IN_BEFORE_PICK")
                pass
            elif BEFORE_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    BEFORE_PICK = False
                    IN_GO_PICK = True
                    pass
                pass
            elif IN_GO_PICK:
                self.go(q_go_pick,t_go_pick)
                self.gripper_pub.publish("0.45")
                IN_GO_PICK = False
                GO_PICK = True
                print("IN_GO_PICK")
                pass
            elif GO_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_PICK = False
                    IN_PICK = True
                    pass
                pass
            elif IN_PICK:
                t_passed = 0.0
                self.gripper_pub.publish("0.57")
                IN_PICK = False
                PICK = True
                print("IN_PICK")
                pass
            elif PICK   :
                if t_passed >= t_pick:
                    PICK = False
                    IN_AFTER_PICK = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_AFTER_PICK:
                self.go(q_after_pick,t_after_pick)
                IN_AFTER_PICK = False
                AFTER_PICK = True
                print("IN_AFTER_PICK")
                pass
            elif AFTER_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    AFTER_PICK = False
                    IN_GO_SHOW = True
                    pass
                pass
            elif IN_GO_SHOW:
                self.go(q_show,t_go_show)
                IN_GO_SHOW = False
                GO_SHOW = True
                print("IN_GO_SHOW")
                pass
            elif GO_SHOW:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_SHOW = False
                    IN_SHOW = True
                    pass
                pass
            elif IN_SHOW:
                t_passed = 0.0
                IN_SHOW = False
                SHOW = True
                print("IN_SHOW")
                pass
            elif SHOW:
                if t_passed >= t_show:
                    SHOW = False
                    IN_BEFORE_RELEASE = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_BEFORE_RELEASE:
                self.go(q_before_release,t_before_release)
                IN_BEFORE_RELEASE = False
                BEFORE_RELEASE = True
                print("IN_BEFORE_RELEASE")
                pass
            elif BEFORE_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    BEFORE_RELEASE = False
                    IN_GO_RELEASE = True
                    pass
                pass
            elif IN_GO_RELEASE:
                self.go(q_release,t_go_release)
                IN_GO_RELEASE = False
                GO_RELEASE = True
                print("IN_GO_RELEASE")
                pass
            elif GO_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_RELEASE = False
                    IN_RELEASE = True
                    pass
                pass
            elif IN_RELEASE:
                t_passed = 0.0
                self.gripper_pub.publish("0.45")
                IN_RELEASE = False
                RELEASE = True
                print("IN_RELEASE")
                pass
            elif RELEASE:
                if t_passed >= t_release:
                    RELEASE = False
                    IN_AFTER_RELEASE = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_AFTER_RELEASE:
                self.go(q_after_release,t_after_release)
                t_passed = 0.0
                IN_AFTER_RELEASE = False
                AFTER_RELEASE = True
                print("IN_AFTER_RELEASE")
                pass
            elif AFTER_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    AFTER_RELEASE = False
                    IN_END = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_END:
                #self.gripper_pub.publish("0.7")
                self.go(q_wait,t_go_end)
                self.gripper_pub.publish("0.7")
                IN_END = False
                END = True
                print("IN_END")
                pass
            elif END    :
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    END = False                    
                    print("Finished machine")
                    break
                    pass
                pass
            else        :   
                #rospy.spin()             
                pass
    def state_machine_only_take(self):
        """State machine of pick place and wait function
        GO_WAIT->WAIT->GO_PICK->PICK->GO_SHOW->SHOW->GO_RELEASE->RELEASE->END
        """
        # Parameters of state machine
        # # Joint states of the pick and place action
        # q_wait = [1.83,1.01,-1.92,-1.18,-2.04,-0.84]
        # q_go_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_show = [1.03,0.83,-1.23,-3.00,-2.10,-1.68]
        # q_release = [-0.37,0.67,-1.21,-1.1,-1.61,-0.85]
        q_wait = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -1.3215044180499476, -1.4791424910174769, -1.658243481312887]
        q_before_pick = [2.19,0.57,-1.07,-1.07,-1.57,-1.04]
        q_go_pick = [2.19,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_pick = q_before_pick
        q_show = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -1.07, -1.4791424910174769, -1.658243481312887]
        q_before_release = [1.12,0.57,-1.07,-1.07,-1.57,-1.04]
        q_release = [1.12,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_release = q_before_release
        # Times
        t_go_wait = 0.8
        t_wait = 0.4
        t_before_pick = 3.4
        t_go_pick = 1.0
        t_gripper_open = 0.3
        t_pick = 0.5
        t_gripper_close = 0.2
        t_after_pick = 1.0
        t_go_show = 2.4
        t_show = 1.6
        t_before_release = 2.8
        t_go_release = 1.0
        t_release = 1.0
        t_after_release = 1.0
        t_go_end = 4.4
        # Other Params
        rate = 20 # Hz
        t_passed = 0.0        
        # States
        GO_WAIT         = False
        WAIT            = False
        GO_PICK         = False
        PICK            = False
        GO_SHOW         = False
        SHOW            = False
        GO_RELEASE      = False
        RELEASE         = False
        END             = False
        IN_GO_WAIT      = True
        IN_WAIT         = False
        IN_BEFORE_PICK  = False
        BEFORE_PICK     = False
        IN_GO_PICK      = False
        IN_PICK         = False
        IN_AFTER_PICK   = False
        AFTER_PICK      = False
        IN_GO_SHOW      = False
        IN_SHOW         = False
        IN_BEFORE_RELEASE= False
        BEFORE_RELEASE   = False
        IN_GO_RELEASE   = False
        IN_RELEASE      = False
        IN_AFTER_RELEASE= False
        AFTER_RELEASE   = False
        IN_END          = False

        # State Machine
        sample = rospy.Rate(rate)
        while not rospy.is_shutdown():            
            sample.sleep()
            if IN_GO_WAIT:
                self.go(q_wait,t_go_wait)
                IN_GO_WAIT = False
                GO_WAIT = True
                print("IN_GO_WAIT")
                pass
            elif GO_WAIT:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_WAIT = False
                    IN_WAIT = True
                    pass
                pass
            elif IN_WAIT:
                t_passed = 0.0
                IN_WAIT = False
                WAIT = True
                print("IN_WAIT")
                pass
            if WAIT:
                if t_passed >= t_wait:
                    WAIT = False
                    IN_BEFORE_PICK = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_BEFORE_PICK:
                self.go(q_before_pick,t_before_pick)
                IN_BEFORE_PICK = False
                BEFORE_PICK = True
                print("IN_BEFORE_PICK")
                pass
            elif BEFORE_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    BEFORE_PICK = False
                    IN_GO_PICK = True
                    pass
                pass
            elif IN_GO_PICK:
                self.go(q_go_pick,t_go_pick)
                self.gripper_pub.publish("0.45")
                IN_GO_PICK = False
                GO_PICK = True
                print("IN_GO_PICK")
                pass
            elif GO_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_PICK = False
                    IN_PICK = True
                    pass
                pass
            elif IN_PICK:
                t_passed = 0.0
                self.gripper_pub.publish("0.57")
                IN_PICK = False
                PICK = True
                print("IN_PICK")
                pass
            elif PICK   :
                if t_passed >= t_pick:
                    PICK = False
                    IN_AFTER_PICK = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_AFTER_PICK:
                self.go(q_after_pick,t_after_pick)
                IN_AFTER_PICK = False
                AFTER_PICK = True
                print("IN_AFTER_PICK")
                pass
            elif AFTER_PICK:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    AFTER_PICK = False
                    IN_GO_SHOW = True
                    pass
                pass
            elif IN_GO_SHOW:
                self.go(q_wait,t_go_show)
                IN_GO_SHOW = False
                GO_SHOW = True
                print("IN_GO_SHOW")
                pass
            elif GO_SHOW:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_SHOW = False
                    IN_SHOW = True
                    break
                    pass
                pass
    def state_machine_put_only(self):
        """State machine of pick place and wait function
        GO_WAIT->WAIT->GO_PICK->PICK->GO_SHOW->SHOW->GO_RELEASE->RELEASE->END
        """
        # Parameters of state machine
        # # Joint states of the pick and place action
        # q_wait = [1.83,1.01,-1.92,-1.18,-2.04,-0.84]
        # q_go_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_pick = [1.53,0.65,-1.08,-1.18,-1.59,-0.07]
        # q_show = [1.03,0.83,-1.23,-3.00,-2.10,-1.68]
        # q_release = [-0.37,0.67,-1.21,-1.1,-1.61,-0.85]
        q_wait = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -1.3215044180499476, -1.4791424910174769, -1.658243481312887]
        q_before_pick = [2.19,0.57,-1.07,-1.07,-1.57,-1.04]
        q_go_pick = [2.19,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_pick = q_before_pick
        q_show = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -1.3215044180499476, -1.4791424910174769, -1.658243481312887]
        q_before_release = [1.12,0.57,-1.07,-1.07,-1.57,-1.04]
        q_release = [1.12,0.68,-1.10,-1.19,-1.57,-1.04]
        q_after_release = q_before_release
        # Times
        t_go_wait = 0.8
        t_wait = 0.4
        t_before_pick = 3.4
        t_go_pick = 1.0
        t_gripper_open = 0.3
        t_pick = 0.5
        t_gripper_close = 0.2
        t_after_pick = 1.0
        t_go_show = 2.4
        t_show = 1.6
        t_before_release = 2.8
        t_go_release = 1.0
        t_release = 1.0
        t_after_release = 1.0
        t_go_end = 4.4
        # Other Params
        rate = 20 # Hz
        t_passed = 0.0        
        # States
        GO_WAIT         = False
        WAIT            = False
        GO_PICK         = False
        PICK            = False
        GO_SHOW         = False
        SHOW            = False
        GO_RELEASE      = False
        RELEASE         = False
        END             = False
        IN_GO_WAIT      = True
        IN_WAIT         = False
        IN_BEFORE_PICK  = False
        BEFORE_PICK     = False
        IN_GO_PICK      = False
        IN_PICK         = False
        IN_AFTER_PICK   = False
        AFTER_PICK      = False
        IN_GO_SHOW      = False
        IN_SHOW         = False
        IN_BEFORE_RELEASE= False
        BEFORE_RELEASE   = False
        IN_GO_RELEASE   = False
        IN_RELEASE      = False
        IN_AFTER_RELEASE= False
        AFTER_RELEASE   = False
        IN_END          = False

        # State Machine
        sample = rospy.Rate(rate)
        while not rospy.is_shutdown():            
            sample.sleep()
            if IN_GO_WAIT:
                self.go(q_wait,t_go_wait)
                IN_GO_WAIT = False
                GO_WAIT = True
                print("IN_GO_WAIT")
                pass
            elif GO_WAIT:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_WAIT = False
                    IN_BEFORE_RELEASE = True
                    pass
                pass
            elif IN_BEFORE_RELEASE:
                self.go(q_before_release,t_before_release)
                IN_BEFORE_RELEASE = False
                BEFORE_RELEASE = True
                print("IN_BEFORE_RELEASE")
                pass
            elif BEFORE_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    BEFORE_RELEASE = False
                    IN_GO_RELEASE = True
                    pass
                pass
            elif IN_GO_RELEASE:
                self.go(q_release,t_go_release)
                IN_GO_RELEASE = False
                GO_RELEASE = True
                print("IN_GO_RELEASE")
                pass
            elif GO_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    GO_RELEASE = False
                    IN_RELEASE = True
                    pass
                pass
            elif IN_RELEASE:
                t_passed = 0.0
                self.gripper_pub.publish("0.45")
                IN_RELEASE = False
                RELEASE = True
                print("IN_RELEASE")
                pass
            elif RELEASE:
                if t_passed >= t_release:
                    RELEASE = False
                    IN_AFTER_RELEASE = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_AFTER_RELEASE:
                self.go(q_after_release,t_after_release)
                t_passed = 0.0
                IN_AFTER_RELEASE = False
                AFTER_RELEASE = True
                print("IN_AFTER_RELEASE")
                pass
            elif AFTER_RELEASE:
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    AFTER_RELEASE = False
                    IN_END = True
                    pass
                else:
                    t_passed += 1.0/rate
                    pass
                pass
            elif IN_END:
                #self.gripper_pub.publish("0.7")
                self.go(q_wait,t_go_end)
                self.gripper_pub.publish("0.7")
                IN_END = False
                END = True
                print("IN_END")
                pass
            elif END    :
                if (self.client.get_state() 
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED):
                    END = False                    
                    print("Finished machine")
                    break
                    pass
                pass
            else        :   
                #rospy.spin()             
                pass

    def go(self,q_to_go,time_to_go):
        """Send robot to calculated joint positions
        """
        self.goal_j.trajectory.points = [
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions = q_to_go, 
                velocities = [0,0,0,0,0,0],
                time_from_start=rospy.Duration(time_to_go))]
        self.client.send_goal(self.goal_j)
    
    def grip_now(self):
        self.start_publisher()
        print ("started publisher")
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.2")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.4")
        # #self.gripper_pub.publish("120")
        # print "published"
        # rospy.sleep(1.0)
        # self.gripper_pub.publish("0.6")
        # print "published"
        rospy.sleep(1.0)
        self.gripper_pub.publish("0.0")
        print ("published")

if __name__ == '__main__':
    try:
        rospy.init_node('pickandplace', anonymous=True)
        #print (a[0](1))
        #rosparam.upload_params('calib_params',a[0])
        
        pp = PickPlace()
        q1 = [2.15,0.99,-2.1,-0.98,-1.57,-1.57]
        #pp.go(q1,1.2)
        #q1 = [0.0,0.0,0.0,0.0,0.0,0.0]
        q1 = [2.34,0.50,-0.76,-1.33,-1.55,-0.88]
        q1 = [2.19,0.68,-1.10,-1.19,-1.57,-1.04]
        q1 = [1.5546956062316895, 1.0360864400863647, -2.09479791322817, -0.67, -1.4791424910174769, -1.658243481312887]
        # q1 = [0.66, 0.76,-1.32,-1.03,-1.57,-2.62]
        # q1 = [1.99,0.89,-0.95,-3.11,-2.02,-1.59]
        # pp.grip_now()
        q1 = [2.948946237564087, 1.2651704549789429, -1.7448294798480433, -1.0737288633929651,
  -1.5695560614215296, -1.71824819246401]
        # pp.go(q1,10.0)
        # pp.state_machine()
        # pp.state_machine_put_only()
        #pp.grip_now()
        #send2take()
        
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
