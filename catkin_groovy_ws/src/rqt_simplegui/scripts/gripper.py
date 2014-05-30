#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import rospy
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

if __name__ == "__main__":
    rospy.init_node('gripper_test_node', anonymous=True)

    name_space = '/l_gripper_controller/gripper_action'
    gripper_client = SimpleActionClient(name_space, GripperCommandAction)
    gripper_client.wait_for_server()

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.position = 0.00 #Opens the gripper, use 0.0 to close it.
    gripper_goal.command.max_effort = 30.0

    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(10.0))
    if (gripper_client.get_state() != GoalStatus.SUCCEEDED):
        rospy.logwarn('Gripper action unsuccessful.')
