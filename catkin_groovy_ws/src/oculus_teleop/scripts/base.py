#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

if __name__ == "__main__":
    rospy.init_node('base_test_node', anonymous=True)

    topic_name = '/base_controller/command'
    base_publisher = rospy.Publisher(topic_name, Twist)

    twist_msg = Twist()
    twist_msg.linear = Vector3(10.0, 0.0, 0.0)
    twist_msg.angular = Vector3(0.0, 0.0, 0.0)
    while True:   
        base_publisher.publish(twist_msg)
