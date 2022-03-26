#!/usr/bin/env python
# 34567891123456789212345678931234567894123456789512345678961234567897123456789
"""Module to give UR5 ability to pick and place
This module uses PEP-8 as coding standard.
"""
import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import std_msgs.msg
import rosparam
import rospy
import os
import trajectory_msgs.msg

class StateNode(object):
    def __init__(self,input):
        (name,goal_type,goal,time,ros_handle) = input
        self.name = name
        self.goal = goal
        self.goal_type = goal_type
        self.time = time
        self.ros_handle = ros_handle
        self.time_passed = 0.0
    def action(self):
        pass
    def finished(self):
        return True

class Maneuver(StateNode):
    def action(self):
        """Send robot to calculated joint positions
        """
        self.goal_type.trajectory.points = [
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions = self.goal, 
                velocities = [0,0,0,0,0,0],
                time_from_start=rospy.Duration(self.time))]
        self.ros_handle.send_goal(self.goal_type)
    def finished(self):
        return (self.ros_handle.get_state()
                == actionlib_msgs.msg.GoalStatus.SUCCEEDED)

class GripperControl(StateNode):
    def action(self):
        """Control Gripper
        """
        self.ros_handle.publish(str(self.goal))
    def finished(self):
        self.ros_handle.publish(str(self.goal))
        if(self.time>self.time_passed):
            self.time_passed += 0.1
            return False
        else:
            return True


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
        self.node_list = []
        self.close = 255
        self.open = 185
        self.pick = 208
        self.start_client()
        self.start_publisher()

    def start_publisher(self):
        self.gripper_pub = rospy.Publisher(
            'gripper_control', std_msgs.msg.String, queue_size=1)

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
        print("Waiting for server ...")
        self.client.wait_for_server()
        print("Connected to server")
        self.goal_j = control_msgs.msg.FollowJointTrajectoryGoal()
        self.goal_j.trajectory = trajectory_msgs.msg.JointTrajectory()
        self.goal_j.trajectory.joint_names = JOINT_NAMES
    
    def state_machine_generic(self):
        # State Machine
        rate = 20 # Hz
        sample = rospy.Rate(rate)

        iter = 0
        curr_state = self.node_list[iter] 
        curr_state.action()      
        while not rospy.is_shutdown():  
            if (iter >= len(self.node_list)-1):
                if curr_state.finished():
                    break       
            if(curr_state.finished()):
                iter += 1
                curr_state = self.node_list[iter]
                curr_state.action()
            sample.sleep()
        self.node_list =[]
    
    def state_machine_creator_1(self):
        ######### start as closed ########
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.2,self.gripper_pub)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),1.0,self.client)))
        ######### go pick ########
        self.node_list.append(Maneuver(("ready_pick1",self.goal_j,
            rosparam.get_param("calib_params/ready_pick1"),4.0,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.0,self.gripper_pub)))
        self.node_list.append(Maneuver(("pick1",self.goal_j,
            rosparam.get_param("calib_params/pick1"),0.7,self.client)))
        self.node_list.append(GripperControl(
            ("pick",self.goal_j,self.pick,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_pick1",self.goal_j,
            rosparam.get_param("calib_params/ready_pick1"),0.7,self.client)))
        ######### go release ########
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),3.0,self.client)))
        self.node_list.append(Maneuver(("release",self.goal_j,
            rosparam.get_param("calib_params/release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),3.0,self.client)))

    def state_machine_creator_2(self):
        ######### start as closed ########
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.2,self.gripper_pub)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),1.0,self.client)))
        ######### go pick ########
        self.node_list.append(Maneuver(("ready_pick2",self.goal_j,
            rosparam.get_param("calib_params/ready_pick2"),4.0,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.0,self.gripper_pub)))
        self.node_list.append(Maneuver(("pick2",self.goal_j,
            rosparam.get_param("calib_params/pick2"),0.7,self.client)))
        self.node_list.append(GripperControl(
            ("pick",self.goal_j,self.pick,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_pick2",self.goal_j,
            rosparam.get_param("calib_params/ready_pick2"),0.7,self.client)))
        ######### go release ########
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),3.0,self.client)))
        self.node_list.append(Maneuver(("release",self.goal_j,
            rosparam.get_param("calib_params/release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),3.0,self.client)))

    def state_machine_creator_3(self):
        ######### start as closed ########
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.2,self.gripper_pub)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),1.0,self.client)))
        ######### go pick ########
        self.node_list.append(Maneuver(("ready_pick3",self.goal_j,
            rosparam.get_param("calib_params/ready_pick3"),4.0,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.0,self.gripper_pub)))
        self.node_list.append(Maneuver(("pick3",self.goal_j,
            rosparam.get_param("calib_params/pick3"),0.7,self.client)))
        self.node_list.append(GripperControl(
            ("pick",self.goal_j,self.pick,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_pick3",self.goal_j,
            rosparam.get_param("calib_params/ready_pick3"),0.7,self.client)))
        ######### go release ########
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),3.0,self.client)))
        self.node_list.append(Maneuver(("release",self.goal_j,
            rosparam.get_param("calib_params/release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),3.0,self.client)))


    def state_machine_creator_4(self):
        ######### start as closed ########
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.2,self.gripper_pub)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),1.0,self.client)))
        ######### go pick ########
        self.node_list.append(Maneuver(("ready_pick4",self.goal_j,
            rosparam.get_param("calib_params/ready_pick4"),4.0,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.0,self.gripper_pub)))
        self.node_list.append(Maneuver(("pick4",self.goal_j,
            rosparam.get_param("calib_params/pick4"),0.7,self.client)))
        self.node_list.append(GripperControl(
            ("pick",self.goal_j,self.pick,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_pick4",self.goal_j,
            rosparam.get_param("calib_params/ready_pick4"),0.7,self.client)))
        ######### go release ########
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),3.0,self.client)))
        self.node_list.append(Maneuver(("release",self.goal_j,
            rosparam.get_param("calib_params/release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("open",self.goal_j,self.open,0.3,self.gripper_pub)))
        self.node_list.append(Maneuver(("ready_release",self.goal_j,
            rosparam.get_param("calib_params/ready_release"),1.2,self.client)))
        self.node_list.append(GripperControl(
            ("close",self.goal_j,self.close,0.0,self.gripper_pub)))
        ######### go wait ########
        self.node_list.append(Maneuver(("wait",self.goal_j,
            rosparam.get_param("calib_params/wait"),3.0,self.client)))

    def go(self,q_to_go,time_to_go):
        """Send robot to calculated joint positions
        """
        self.goal_j.trajectory.points = [
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions = q_to_go, 
                velocities = [0,0,0,0,0,0],
                time_from_start=rospy.Duration(time_to_go))]
        self.client.send_goal(self.goal_j)

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
        pp.go(rosparam.get_param("calib_params/wait"),10.0)
        #pp.go(rosparam.get_param("calib_params/ready_pick4"),10.0)
        #pp.go(rosparam.get_param("calib_params/ready_release"),10.0)
        #pp.go(rosparam.get_param("calib_params/release"),10.0)
        #pp.go(rosparam.get_param("calib_params/ready_pick2"),10.0)
        # pp.state_machine()
        # pp.state_machine_put_only()
        #pp.grip_now()
        # #send2take()
        # pp.state_machine_creator_1()
        # pp.state_machine_generic()
        # pp.state_machine_creator_2()
        # pp.state_machine_generic()
        # pp.state_machine_creator_3()
        # pp.state_machine_generic()
        # pp.state_machine_creator_4()
        # pp.state_machine_generic()
        
        
        
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
