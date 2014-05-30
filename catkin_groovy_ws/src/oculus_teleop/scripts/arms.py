#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

from subprocess import call
import threading
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient

def main():
    rospy.init_node('arm_test_node', anonymous=True)
    #joint_sig = Signal(JointState)
    #joint_sig.connect(joint_sig_cb)
    #lock = threading.Lock()
    #rospy.Subscriber('joint_states', JointState, joint_states_cb)

    r_joint_names = ['r_shoulder_pan_joint',
                          'r_shoulder_lift_joint',
                          'r_upper_arm_roll_joint',
                          'r_elbow_flex_joint',
                          'r_forearm_roll_joint',
                          'r_wrist_flex_joint',
                          'r_wrist_roll_joint']
    l_joint_names = ['l_shoulder_pan_joint',
                          'l_shoulder_lift_joint',
                          'l_upper_arm_roll_joint',
                          'l_elbow_flex_joint',
                          'l_forearm_roll_joint',
                          'l_wrist_flex_joint',
                          'l_wrist_roll_joint']


    # Create a trajectory action client
    r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
    r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
    rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
    r_traj_action_client.wait_for_server()
    l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
    l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
    rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
    l_traj_action_client.wait_for_server()
    

    

    '''Moves the arm to the desired joints'''
    positions = [-1.5,0.0,0.0,0.0,0.0,0.0,0.0]
    velocities = [0] * len(positions)
    traj_goal = JointTrajectoryGoal()
    traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
    traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=positions, velocities=velocities, time_from_start=rospy.Duration(0.5)))
    traj_goal.trajectory.joint_names = r_joint_names

    r_traj_action_client.send_goal(traj_goal)

if __name__ == "__main__":
    main()
