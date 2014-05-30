#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('pr2_mechanism_msgs')

from subprocess import call
import rospy
from pr2_mechanism_msgs.srv import SwitchController


class SwitchNode():
    
    def __init__(self):
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name, SwitchController)

    def switch(self, start_controllers, stop_controllers, num):
        self.switch_service_client(start_controllers, stop_controllers, num)
