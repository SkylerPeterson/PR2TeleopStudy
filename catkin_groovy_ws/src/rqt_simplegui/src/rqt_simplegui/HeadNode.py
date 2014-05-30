#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('control_msgs')

from subprocess import call
import math
import threading
import rospy
from geometry_msgs.msg import Point
from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


class HeadNode():
    
    def __init__(self):
        name_space = '/head_traj_controller/point_head_action'
        self.head_client = SimpleActionClient(name_space, PointHeadAction)
        self.head_client.wait_for_server()
	
    def moveHead(self, theta, phi):
        
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'base_link'
        head_goal.min_duration = rospy.Duration(1.0)
        head_goal.target.point = Point(math.cos(theta), math.sin(theta), phi)
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(10.0))
        if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Head action unsuccessful.')
