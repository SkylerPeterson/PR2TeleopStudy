#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

from subprocess import call
import threading
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient


class ArmNode():
    
    # Folding the arms, positions
    R_FOLD_POS_1 = [-0., -0.4, -0., -1.3, 0., -0.1, 0.]
    L_FOLD_POS_1 = [ 0.,  1.3, -0.1, -2.1, 0., -0.1, 0.]
    R_FOLD_POS_2 = [-0., -0.4, -1.5, -1.3, -0., -0.1, -0.]
    #L_FOLD_POS_2 = [-0.,  1.3, 1.7, -1.8, 1.4, -0.1, 0.]
    L_FOLD_POS_2 = [-0.,  1.3, 1.7, -1.8, 4.9, -0.1, -4.68]
    R_FOLD_POS_3 = [-0., 1.1, -1.4, -1.9, -1.5, -0.1, -0.]
    #L_FOLD_POS_3 = [-0., 1.3, 1.7, -1.8, 1.4, -0.1, 0.]
    L_FOLD_POS_3 = [-0., 1.3, 1.8, -1.8, 4.9, -0.1, -4.68]
    R_FOLD_POS_4 = [-0., 1.2, 0., -2., -0., -0.1, -0.]
    # For stacking
    R_STACK_INIT = [0.0, -0.4, -3.1, -0.9, -18.9, -1.1, -18.9]
    
    def __init__(self):

        
        self.r_joint_names = ['r_shoulder_pan_joint',
                              'r_shoulder_lift_joint',
                              'r_upper_arm_roll_joint',
                              'r_elbow_flex_joint',
                              'r_forearm_roll_joint',
                              'r_wrist_flex_joint',
                              'r_wrist_roll_joint']
        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']
        self.all_joint_names = []
        self.all_joint_poses = []

        self.lock = threading.Lock()

        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
	l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
            joint_states message is received'''
        self.lock.acquire()
        self.setArmNames(msg.name)
        self.setArmPositions(msg.position)
        self.lock.release()

    def joint_sig_cb(self, msg):
	pass

    def setArmNames(self, names):
        self.all_joint_names = names

    def setArmPositions(self, pos):
        self.all_joint_poses = pos

    def set_arm_mode(self, start_controllers, stop_controllers, swnode):
        try:
            swnode.switch(start_controllers, stop_controllers, 1)
        except rospy.ServiceException:
            rospy.logerr('Could not change arm mode.')

    def get_joint_state(self, side_prefix):
        '''Returns position for arm joints on the requested side (r/l)'''
        if side_prefix == 'r':
            joint_names = self.r_joint_names
        else:
            joint_names = self.l_joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received yet!\n")
            return None
    
        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
                self.lock.release()
                return None

        self.lock.release()
        return positions

    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        velocities = [0] * len(positions)
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=positions,
                            velocities=velocities, time_from_start=rospy.Duration(time_to_joint)))
	
	if (side_prefix == 'r'):
	    traj_goal.trajectory.joint_names = self.r_joint_names
	    self.r_traj_action_client.send_goal(traj_goal)
	else:
	    traj_goal.trajectory.joint_names = self.l_joint_names
	    self.l_traj_action_client.send_goal(traj_goal)

    def fold_motion(self):
        '''Moves the arm to the desired joints'''
        velocities = [0,0,0,0,0,0,0]
        # Right arm movements
        r_traj_goal = JointTrajectoryGoal()
        r_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_FOLD_POS_1, velocities=velocities, time_from_start=rospy.Duration(2.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_FOLD_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_FOLD_POS_3, velocities=velocities, time_from_start=rospy.Duration(6.0)))
	r_traj_goal.trajectory.joint_names = self.r_joint_names
        # Left arm movements
        l_traj_goal = JointTrajectoryGoal()
        l_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_1, velocities=velocities, time_from_start=rospy.Duration(2.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_3, velocities=velocities, time_from_start=rospy.Duration(6.0)))
        l_traj_goal.trajectory.joint_names = self.l_joint_names
        # Set goal
	self.r_traj_action_client.send_goal(r_traj_goal)
	self.l_traj_action_client.send_goal(l_traj_goal)
	
    def stack_init_motion(self):
        '''Moves the arm to the desired joints'''
        velocities = [0,0,0,0,0,0,0]
        # Right arm movements
        r_traj_goal = JointTrajectoryGoal()
        r_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_STACK_INIT, velocities=velocities, time_from_start=rospy.Duration(2.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_STACK_INIT, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_STACK_INIT, velocities=velocities, time_from_start=rospy.Duration(6.0)))
	r_traj_goal.trajectory.joint_names = self.r_joint_names
        # Left arm movements
        l_traj_goal = JointTrajectoryGoal()
        l_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_1, velocities=velocities, time_from_start=rospy.Duration(2.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_FOLD_POS_3, velocities=velocities, time_from_start=rospy.Duration(6.0)))
        l_traj_goal.trajectory.joint_names = self.l_joint_names
        # Set goal
	self.r_traj_action_client.send_goal(r_traj_goal)
	self.l_traj_action_client.send_goal(l_traj_goal)
    
            
