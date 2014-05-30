#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('control_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')

from subprocess import call
import threading
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class GripperNode():
    
    def __init__(self):
        
        name_space = '/l_gripper_controller/gripper_action'
        self.l_gripper_client = SimpleActionClient(name_space, GripperCommandAction)
        self.l_gripper_client.wait_for_server()
        name_space = '/r_gripper_controller/gripper_action'
        self.r_gripper_client = SimpleActionClient(name_space, GripperCommandAction)
        self.r_gripper_client.wait_for_server()
	

    def changeGripper(self, side_prefix, pos):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.position = pos
        gripper_goal.command.max_effort = 30.0

        if side_prefix == 'l':
            self.l_gripper_client.send_goal(gripper_goal)
            self.l_gripper_client.wait_for_result(rospy.Duration(10.0))
        elif side_prefix == 'r':
            self.r_gripper_client.send_goal(gripper_goal)
            self.r_gripper_client.wait_for_result(rospy.Duration(10.0))

        if (self.l_gripper_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Gripper action unsuccessful.')
	    
    
