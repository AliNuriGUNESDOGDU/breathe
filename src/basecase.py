#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import argparse
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import *
import matplotlib.pyplot as plt
import numpy as np
import os
import subprocess

def forwardkinemodel(q1,q2):
    a1 = 0.42500
    a2 = 0.39225
    y = a1*math.sin(q1)+a2*math.sin(q1+q2)
    x = a1*math.cos(q1)+a2*math.cos(q1+q2)
    return x,y
def inversekinematicmodel(x,y):
    a1 = 0.42500
    a2 = 0.39225
    q2 = -math.acos((x*x+y*y-a1*a1-a2*a2)/2/a1/a2)
    q1 = math.atan(y/x)-math.atan((a2*math.sin(q2))/(a1+a2*math.cos(q2)))
    return q1,q2

def gripper_client(value,link):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q1 = [2.2,0,-1.57,0,0,0]
    Q2 = [1.5,0,-1.57,0,0,0]
    Q3 = [1.5,-0.2,-1.57,0,0,0]
    Q4 = [0.15,0,0,0,0,0]
    if rospy.has_param('arm_controller'):
    	namespace_ = '/arm_controller/follow_joint_trajectory'
    else:
    	namespace_ = '/follow_joint_trajectory'
    # Create an action client
    client = actionlib.SimpleActionClient(
        namespace_,  # namespace of the action topics
        control_msgs.msg.FollowJointTrajectoryAction # action type
    )
    print "Waiting for server..."        
    client.wait_for_server()
    print "Connected to server"
    g = control_msgs.msg.FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    d = 2.0
    g.trajectory.points = [
        JointTrajectoryPoint(positions=[1.5708,0,0,-2.58,-1.5708,1.5708], velocities=[0.01,0,0,0,0,0], 
            time_from_start=rospy.Duration(1.0))]

    video_record = False
    '''
    # Breathe Type-1
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.02   
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-2
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 30
    amplitude = 0.02
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-3
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 6
    amplitude = 0.02
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-4
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.1
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-5
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 6
    amplitude = 0.1
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-6
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.0
    phase_angle_y = 0.0
    bpm = 30
    amplitude = 0.1
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-7
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 1.57
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.02
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-8
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 1.57
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.1
    y_kp = 1.0 
    #'''

    '''
    # Breathe Type-9
    x_init = -0.2
    y_init = 0.157
    angle = 0.0
    phase_angle_x = 0.78
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.02
    y_kp = 1.0 
    #'''

    #'''
    # Breathe Type-10
    x_init = -0.2
    y_init = 0.157
    phase_angle_x = 0.78
    phase_angle_y = 0.0
    bpm = 15
    amplitude = 0.1
    y_kp = 1.0 
    #'''

  





    




    numberofsamples = round(600/bpm)
    angle = 0.0
    
    save_string = "bpm_"+ str(bpm) + "_ampl_" +str(amplitude) + "_px_" \
        + str(phase_angle_x) + "_py_" + str(phase_angle_y) +"_ykp_" +str(y_kp)


    while d < 3:
        x1 = x_init + amplitude*math.cos(angle+phase_angle_x)
        y1 = y_init + amplitude*y_kp*math.cos(angle+phase_angle_y)
        angle += math.pi/numberofsamples
        d += 0.1
        q1,q2 = inversekinematicmodel(x1,y1)
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=[0,q1,q2,-2.58-q1-q2,-1.5708,1.5708], 
                velocities=[0.01,0,0,0,0,0], time_from_start=rospy.Duration(d)))

    client.send_goal(g)

    # tmplogdir = "/home/ali/catkin_ws/src/universal_robot-kinetic-devel/ur_gazebo/logtmp/"

    # video_string_side =  save_string + "side" + ".mp4"
    # video_string_front = save_string + "front" + ".mp4"

    # rospy.Rate(1)

    # rospy.sleep(40)

    
    # os.system("mkdir logtmp")

    


    # record_command1 = "ffmpeg -r 15 -pattern_type glob -i '/tmp/camera_save_tutorial/ur5_cubes_front_camera_model_link_front_camera*.jpg' -c:v libx264 " \
    #     +tmplogdir+ video_string_front

    # record_command2 = "ffmpeg -r 15 -pattern_type glob -i '/tmp/camera_save_tutorial/ur5_cubes_side_camera_model_link_side_camera*.jpg' -c:v libx264 " \
    #     +tmplogdir+ video_string_side

    # print "*/*/*/*/*/*/*/*/ Recording /*/*/*/*/*/*/*/*/"

    # os.system(record_command1)
    # os.system(record_command2)
    # rospy.sleep(5)

    # print "*/*/*/*/*/*/*/*/ Recording Finished /*/*/*/*/*/*/*/*/"

    # print "*/*/*/*/*/*/*/*/ Cutting /*/*/*/*/*/*/*/*/"
    # os.system("ffmpeg -i "+tmplogdir +video_string_side+" -ss 00:20 -to 00:45 "+tmplogdir +"cut_"+video_string_side)
    # os.system("ffmpeg -i "+tmplogdir+video_string_front+" -ss 00:20 -to 00:45 "+tmplogdir +"cut_"+video_string_front)
    # rospy.sleep(5)
    # print "*/*/*/*/*/*/*/*/ Cutting Finished /*/*/*/*/*/*/*/*/"

    # print "*/*/*/*/*/*/*/*/ Merging /*/*/*/*/*/*/*/*/"
    # os.system("ffmpeg -i "+tmplogdir +"cut_"+video_string_front+ " -i "+tmplogdir +"cut_"+video_string_side+ " -filter_complex "
    # +"\"[0]pad=iw+5:color=black[left];[left][1]hstack=inputs=2\""+" combined_" +save_string + ".mp4")...
    # print "*/*/*/*/*/*/*/*/ Merging Finished /*/*/*/*/*/*/*/*/"
    # os.system("mv " +" combined_" +save_string + ".mp4" " /home/ali/catkin_ws/src/universal_robot-kinetic-devel/ur_gazebo/log")
    
    # os.system("rm -r -d " + tmplogdir)
    # # p = subprocess.call(record_command1)
    # # p.wait()
    # # p = subprocess.call(record_command2)
    # # p.wait()

    # # p = subprocess.call("ffmpeg -i "+video_string_side+" -ss 00:15 -to 00:25 cut_"+video_string_side)
    # # p.wait()

    # # p = subprocess.call("ffmpeg -i "+video_string_front+" -ss 00:15 -to 00:25 cut_"+video_string_front)
    # # p.wait()

    # # p = subprocess.call("ffmpeg -i cut_"+ video_string_front + " -i cut_"+ video_string_front + " -filter_complex "+"\"[0]pad=iw+5:color=black[left];[left][1]hstack=inputs=2\""+" combined_" +save_string)
    # # p.wait()







if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.7",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        parser.add_argument("--link", type=float, default="0.7",
                            help="Value between 0.0 (open) and 0.7 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        link_value = args.link
        # Start the ROS node
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        # Set the value to the gripper
        # positions,velocities= twospringmodel(1.0)
        # arr_pos = np.array(positions)
        # arr_vel = np.array(velocities)
        # plt.plot(arr_pos)
        # plt.show()
        # plt.plot(arr_vel)
        # plt.show()
        result = gripper_client(gripper_value,link_value)
         
        # print forwardkinemodel (0.115,0.749)
        # print inversekinematicmodel(0.1,0.4)
        # x1 = -0.2
        # y1 = 0.347
        # for i in range(20):
        #     x1 = 0.005 +x1
        #     y1 = 0.005 +y1
        #     print "real"
        #     print (x1,y1)
        #     print "from inverse"
        #     print inversekinematicmodel(x1,y1)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")




