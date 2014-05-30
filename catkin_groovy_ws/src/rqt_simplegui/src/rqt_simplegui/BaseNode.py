#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')

from subprocess import call
import threading
import rospy
from geometry_msgs.msg import Vector3, Twist


class BodyNode():
    
    def __init__(self):
        name_space = '/base_controller/command'
        self.base_pub = rospy.Publisher(name_space, Twist)
	
    def twistBody(self, x, y, z):
        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        twist_msg.angular = Vector3(x, y, z)
        for i in range(100):
            self.base_pub.publish(twist_msg)
	
    def moveBody(self, x, y, z):
        twist_msg = Twist()
        twist_msg.linear = Vector3(x, y, z)
        twist_msg.angular = Vector3(0.0, 0.0, 0.0)
        for i in range(100):
            self.base_pub.publish(twist_msg)
	
    
