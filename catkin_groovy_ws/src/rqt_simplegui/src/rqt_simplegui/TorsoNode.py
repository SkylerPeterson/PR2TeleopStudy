#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('pr2_controllers_msgs')

from subprocess import call
import actionlib
import threading
import rospy
import pr2_controllers_msgs.msg


class TorsoNode():
    
    def __init__(self):
        name_space = '/torso_controller/position_joint_action'
        self.client = actionlib.SimpleActionClient(name_space, pr2_controllers_msgs.msg.SingleJointPositionAction)
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()
    
    def lowerTorso(self):
        # Creates a goal to send to the action server.
        goal = pr2_controllers_msgs.msg.SingleJointPositionGoal()
        goal.position=0.0
        goal.min_duration=rospy.Duration(2.0)
        goal.max_velocity=1.0
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        # self.client.wait_for_result()
        
        # Prints out the result of executing the action
        # return self.client.get_result()
    
    def raiseTorso(self):
        # Creates a goal to send to the action server.
        goal = pr2_controllers_msgs.msg.SingleJointPositionGoal()
        goal.position=1.95
        goal.min_duration=rospy.Duration(2.0)
        goal.max_velocity=1.0
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        #self.client.wait_for_result()
        
        # Prints out the result of executing the action
        #return self.client.get_result()
