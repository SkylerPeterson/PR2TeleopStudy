#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sound_play')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('trajectory_msgs')


from subprocess import call
import threading
import rospy
import math
import numpy as np
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame, QGroupBox, QInputDialog, QMessageBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal, QRect
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal, JointTrajectoryAction, JointTrajectoryGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState


class SimpleGUI(Plugin):

    sound_sig = Signal(SoundRequest)
    joint_sig = Signal(JointState)
    # Constants to help us move the robot
    HEAD_THETA = 0.0
    HEAD_Z = 1.0
    LEFT_GRIPPER_POSITION = 0.0
    RIGHT_GRIPPER_POSITION = 0.0
    GRIPPER_INTENSITY = 0.0
    LINEAR_X = 0.0
    LINEAR_Y = 0.0
    LINEAR_Z = 0.0
    ANGULAR_X = 0.0
    ANGULAR_Y = 0.0
    ANGULAR_Z = 0.0
    MOTION_INTENSITY = 0.0
    ARM_INTENSITY = 0.0
    # Wave the arm, positions
    R_WAVE_POS_1 = [0.0, 1.3, 0.0, -1.1, 0.0, -0.1, 0.0]
    L_WAVE_POS_1 = [1.7, 0.0, 0.0, -1.2, 0.0, -0.1, 1.7]
    L_WAVE_POS_2 = [1.7, 0.0, 0.0, -2.1, 0.0, -0.1, 1.7]
    # Clap hands
    R_CLAP_POS_1 = [-0.1, 0.3, -0.4, -1.9, 1.5, -0.4, -0.1]
    L_CLAP_POS_1 = [ 0.1, 0.3, 0.4, -1.9, -1.5, -0.4, 3.4]
    R_CLAP_POS_2 = [-0.1, 0.3, -0.7, -1.9, 1.5, -0.7, -0.1]
    L_CLAP_POS_2 = [ 0.2, 0.2, 0.9, -1.9, -1.5, -0.7, 3.2]

    r_arm_pose_box = QtGui.QComboBox()
    l_arm_pose_box = QtGui.QComboBox()

    def __init__(self, context):
        super(SimpleGUI, self).__init__(context)
        self.setObjectName('SimpleGUI')
        self._widget = QWidget()

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
        self.saved_r_arm_pose = dict()
        self.saved_l_arm_pose = dict()

        self.lock = threading.Lock()
        
        # Add subscribers
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name,
                                                 SwitchController)

        # Add Clients
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        name_space = '/head_traj_controller/point_head_action'
        self.head_client = SimpleActionClient(name_space, PointHeadAction)
        self.head_client.wait_for_server()
        name_space = '/l_gripper_controller/gripper_action'
        self.l_gripper_client = SimpleActionClient(name_space, GripperCommandAction)
        self.l_gripper_client.wait_for_server()
        name_space = '/r_gripper_controller/gripper_action'
        self.r_gripper_client = SimpleActionClient(name_space, GripperCommandAction)
        self.r_gripper_client.wait_for_server()
	name_space = '/base_controller/command'
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
	l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()
  	self.base_pub = rospy.Publisher(name_space, Twist)
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        # Main layout element for GUI
        large_box = QtGui.QVBoxLayout()
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        

        # Create Button box for GUI
	button_box = QtGui.QGridLayout()
        # Add The Buttons to button box here
        button_box.addWidget(self.create_button('Speak'), 2, 5)
        button_box.addWidget(self.create_button('Head ^'), 1, 5)
        button_box.addWidget(self.create_button('Head v'), 3, 5)
        button_box.addWidget(self.create_button('Head >'), 2, 6)
        button_box.addWidget(self.create_button('< Head'), 2, 4)
        button_box.addWidget(self.create_button('LShldr Pan Out'), 5, 2)
	button_box.addWidget(self.create_button('LShldr Pan In'), 5, 3)
        button_box.addWidget(self.create_button('LShldr Lift ^'), 6, 2)
	button_box.addWidget(self.create_button('LShldr Lift v'), 6, 3)
        button_box.addWidget(self.create_button('LUpArm Roll Out'), 7, 2)
        button_box.addWidget(self.create_button('LUpArm Roll In'), 7, 3)
	button_box.addWidget(self.create_button('LElbo Flex ^'), 8, 2)
        button_box.addWidget(self.create_button('LElbo Flex v'), 8, 3)
        button_box.addWidget(self.create_button('LLoArm Roll Out'), 9, 2)
        button_box.addWidget(self.create_button('LLoArm Roll In'), 9, 3)
	button_box.addWidget(self.create_button('LWrst Flex ^'), 10, 2)
        button_box.addWidget(self.create_button('LWrst Flex v'), 10, 3)
        button_box.addWidget(self.create_button('LWrst Roll Out'), 11, 2)
        button_box.addWidget(self.create_button('LWrst Roll In'), 11, 3)
	button_box.addWidget(self.create_button('LGr Close'), 12, 2)
        button_box.addWidget(self.create_button('LGr Open'), 12, 3)
        button_box.addWidget(self.create_button('RShldr Pan Out'), 5, 8)
	button_box.addWidget(self.create_button('RShldr Pan In'), 5, 7)
        button_box.addWidget(self.create_button('RShldr Lift ^'), 6, 8)
	button_box.addWidget(self.create_button('RShldr Lift v'), 6, 7)
        button_box.addWidget(self.create_button('RUpArm Roll Out'), 7, 8)
        button_box.addWidget(self.create_button('RUpArm Roll In'), 7, 7)
	button_box.addWidget(self.create_button('RElbo Flex ^'), 8, 8)
        button_box.addWidget(self.create_button('RElbo Flex v'), 8, 7)
        button_box.addWidget(self.create_button('RLoArm Roll Out'), 9, 8)
        button_box.addWidget(self.create_button('RLoArm Roll In'), 9, 7)
	button_box.addWidget(self.create_button('RWrst Flex ^'), 10, 8)
        button_box.addWidget(self.create_button('RWrst Flex v'), 10, 7)
        button_box.addWidget(self.create_button('RWrst Roll Out'), 11, 8)
        button_box.addWidget(self.create_button('RWrst Roll In'), 11, 7)
        button_box.addWidget(self.create_button('RGr Close'), 12, 8)
        button_box.addWidget(self.create_button('RGr Open'), 12, 7)
	button_box.addWidget(self.create_button('Twist >'), 11, 6)
        button_box.addWidget(self.create_button('< Twist'), 11, 4)        
	button_box.addWidget(self.create_button('Move ^'), 10, 5)
        button_box.addWidget(self.create_button('Move v'), 13, 5)
	button_box.addWidget(self.create_button('< Move'), 12, 4)
        button_box.addWidget(self.create_button('Move >'), 12, 6)

        # Add Button Box to main GUI layout element
        large_box.addLayout(button_box)
        large_box.addItem(QtGui.QSpacerItem(100,20)) 
	
        # Create and add slider box
        gripper_box = QtGui.QHBoxLayout()
        self.gripper_label = QtGui.QLabel('Gripper Intensity: ')
        self.gripper_label.setPalette(palette)
        gripper_box.addWidget(self.gripper_label)
        gripper_box.addWidget(self.create_horizontal_slider('Gripper_Slider'))
	large_box.addLayout(gripper_box)        
        
        # Create and add slider box
        motion_box = QtGui.QHBoxLayout()
        self.motion_label = QtGui.QLabel('Motion Intensity: ')
        self.motion_label.setPalette(palette)
        motion_box.addWidget(self.motion_label)
        motion_box.addWidget(self.create_horizontal_slider('Motion_Slider'))       
	large_box.addLayout(motion_box)

        # Create and add slider box
        arm_box = QtGui.QHBoxLayout()
        self.arm_label = QtGui.QLabel('Arm Intensity: ')
        self.arm_label.setPalette(palette)
        arm_box.addWidget(self.arm_label)
        arm_box.addWidget(self.create_horizontal_slider('Arm_Slider'))       
	large_box.addLayout(arm_box)

        # Speech box for speech display created
        speech_box = QtGui.QVBoxLayout()
        self.speech_label = QtGui.QLabel('Robot is ready to run, press button to start')
        self.speech_label.setPalette(palette)
        speech_box.addWidget(self.speech_label)
        # Speech box added to main layout
        large_box.addLayout(speech_box)

        # Speech box for speech display created
        speech_box_2 = QtGui.QHBoxLayout()
        self.speech_label_2 = QtGui.QLabel('Enter Speech Text: ')
        self.speech_label_2.setPalette(palette)
        speech_box_2.addWidget(self.speech_label_2)
        self.text_in = QtGui.QLineEdit('Thank You')
        speech_box_2.addWidget(self.text_in)
        # Speech box added to main layout
        large_box.addLayout(speech_box_2)
        large_box.addItem(QtGui.QSpacerItem(100,20))
        
        # Add arm freeze/Relax state buttons
        button_box1 = QtGui.QHBoxLayout()
        button_box1.addWidget(self.create_button('Relax right arm'))
        button_box1.addWidget(self.create_button('Freeze right arm'))
        button_box1.addWidget(self.create_button('Relax left arm'))
        button_box1.addWidget(self.create_button('Freeze left arm'))
        large_box.addLayout(button_box1)

        # Add Arm Pose Buttons
        button_box2 = QtGui.QHBoxLayout()
        button_box2.addWidget(self.create_button('Save right arm pose'))
        button_box2.addWidget(self.create_button('Save left arm pose'))
        button_box2.addWidget(self.create_button('Move right arm to selected pose'))
        button_box2.addWidget(self.create_button('Move left arm to selected pose'))
        button_box2.addWidget(self.create_button('Delete selected right arm pose'))
        button_box2.addWidget(self.create_button('Delete selected left arm pose'))
        large_box.addLayout(button_box2)

        # Add predefined arm movements buttons
        button_box3 = QtGui.QHBoxLayout()
        button_box3.addWidget(self.create_button('Wave Arm'))
        button_box3.addWidget(self.create_button('Clap Hands'))
        button_box3.addStretch(1)
        large_box.addLayout(button_box3)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        # Add temp arm pose drop boxes
        state_box = QtGui.QGridLayout()
        self.r_arm_label = QtGui.QLabel('Saved Right Arm Poses')
        self.r_arm_label.setPalette(palette)
        state_box.addWidget(self.r_arm_label, 1, 1)
        state_box.addWidget(self.r_arm_pose_box, 2, 1)
        self.l_arm_label = QtGui.QLabel('Saved Left Arm Poses')
        self.l_arm_label.setPalette(palette)
        state_box.addWidget(self.l_arm_label, 1, 2)
        state_box.addWidget(self.l_arm_pose_box, 2, 2)
        large_box.addLayout(state_box)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        # Add create arm movements from pose buttons
        button_box3 = QtGui.QHBoxLayout()
        self.arm_move_label = QtGui.QLabel('Saved Arm Movements:')
        self.arm_move_label.setPalette(palette)
        button_box3.addWidget(self.arm_move_label)
        self.arm_move_box = QtGui.QComboBox()
        button_box3.addWidget(self.arm_move_box)
        button_box3.addWidget(self.create_button('Create arm movement'))
        button_box3.addWidget(self.create_button('Initiate selected arm movement'))
        button_box3.addWidget(self.create_button('Delete arm movement'))
        large_box.addLayout(button_box3)

        # Puts layout in widget and adds widget name for RosGUI to identify
        self._widget.setObjectName('SimpleGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        rospy.loginfo('GUI initialization complete.')
        
    # Triggered When Subscribber to robot sound detects a message
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    # Will create a button, with all buttons using same event trigger
    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(self.command_cb)
        btn.setAutoRepeat(True)
        return btn

    # Will create a horizontal slider
    def create_horizontal_slider(self, slider_name):
	slider = QtGui.QSlider(1)
        if (slider_name == 'Gripper_Slider'):
	    slider.valueChanged.connect(self.gripper_slider)
        elif (slider_name == 'Motion_Slider'):
            slider.valueChanged.connect(self.motion_slider)
        elif (slider_name == 'Arm_Slider'):
            slider.valueChanged.connect(self.arm_slider)
        slider.setMinimum(0)
        slider.setMaximum(1000)
	return slider

    # Triggered when request to sound_play is made
    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        #if (sound_request.command == SoundRequest.SAY):
        qWarning('Robot said: ' + sound_request.arg)
        # self.speech_label.setText('Robot said: ' + sound_request.arg)

    def gripper_slider(self):
	self.GRIPPER_INTENSITY = float(self._widget.sender().sliderPosition())/10000.0
        
    def motion_slider(self):
        self.MOTION_INTENSITY = float(self._widget.sender().sliderPosition())/1000.0

    def arm_slider(self):
        self.ARM_INTENSITY = float(self._widget.sender().sliderPosition())/1000.0

    def show_text_in_rviz(self, text):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, id=0,
                    lifetime=rospy.Duration(1.5),
                    pose=Pose(Point(0.5, 0.5, 1.5), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.06, 0.06, 0.06),
                    header=Header(frame_id='base_link'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self.marker_publisher.publish(marker)

    def save_pose(self, side_prefix):
        text, ok = QInputDialog.getText(None, str('New Arm ' + side_prefix + ' pose'), str('Enter Name of new pose: '))
        if ok:
            if (side_prefix == 'r'):
               self.r_arm_pose_box.addItem(text)
               self.saved_r_arm_pose[text] = self.get_joint_state('r')
               print('Saved right arm pose: ' + str(np.round(self.saved_r_arm_pose[text], 1)))
            else:
               self.l_arm_pose_box.addItem(text)
               self.saved_l_arm_pose[text] = self.get_joint_state('l')
               print('Saved left arm pose: ' + str(np.round(self.saved_l_arm_pose[text], 1)))
    
    def delete_pose(self, side_prefix):
        if (side_prefix == 'r'):
            if len(self.saved_r_arm_pose) > 0:
                del self.saved_r_arm_pose[self.r_arm_pose_box.currentText()]
                self.r_arm_pose_box.removeItem(self.r_arm_pose_box.findText(self.r_arm_pose_box.currentText()))
        else:
            if len(self.saved_l_arm_pose) > 0:
                del self.saved_l_arm_pose[self.l_arm_pose_box.currentText()]
                self.l_arm_pose_box.removeItem(self.l_arm_pose_box.findText(self.l_arm_pose_box.currentText()))
            
    def move_arm(self, side_prefix):
        if (side_prefix == 'r'):
            if len(self.saved_r_arm_pose) == 0:
                rospy.logerr('There are no right arm poses saved yet, cannot move.')
                errBox = QMessageBox()
                errBox.setWindowTitle('Error')
                errBox.setText('There are no right arm poses saved yet, cannot move.')
                errBox.exec_()
            else:
                self.freeze_arm('r')
                self.move_to_joints('r', self.saved_r_arm_pose[self.r_arm_pose_box.currentText()], 2.0)
        else:
            if len(self.saved_l_arm_pose) == 0:
                rospy.logerr('There are no left arm poses saved yet, cannot move.')
                errBox = QMessageBox()
                errBox.setWindowTitle('Error')
                errBox.setText('There are no left arm poses saved yet, cannot move.')
                errBox.exec_()
            else:
                self.freeze_arm('l')
                self.move_to_joints('l', self.saved_l_arm_pose[self.l_arm_pose_box.currentText()], 2.0)
                pass

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

    def wave_motion(self):
        '''Moves the arm to the desired joints'''
        velocities = [0,0,0,0,0,0,0]
        # Right arm movements
        r_traj_goal = JointTrajectoryGoal()
        r_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_WAVE_POS_1, velocities=velocities, time_from_start=rospy.Duration(1.0)))
	r_traj_goal.trajectory.joint_names = self.r_joint_names
        # Left arm movements
        l_traj_goal = JointTrajectoryGoal()
        l_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_WAVE_POS_1, velocities=velocities, time_from_start=rospy.Duration(1.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_WAVE_POS_2, velocities=velocities, time_from_start=rospy.Duration(2.0)))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_WAVE_POS_1, velocities=velocities, time_from_start=rospy.Duration(3.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_WAVE_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        l_traj_goal.trajectory.joint_names = self.l_joint_names
        # Set goal
	self.r_traj_action_client.send_goal(r_traj_goal)
	self.l_traj_action_client.send_goal(l_traj_goal)
	
    def clap_motion(self):
        '''Moves the arm to the desired joints'''
        velocities = [0,0,0,0,0,0,0]
        # Right arm movements
        r_traj_goal = JointTrajectoryGoal()
        r_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_CLAP_POS_1, velocities=velocities, time_from_start=rospy.Duration(1.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_CLAP_POS_2, velocities=velocities, time_from_start=rospy.Duration(2.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_CLAP_POS_1, velocities=velocities, time_from_start=rospy.Duration(3.0)))
        r_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.R_CLAP_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
	r_traj_goal.trajectory.joint_names = self.r_joint_names
        # Left arm movements
        l_traj_goal = JointTrajectoryGoal()
        l_traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_CLAP_POS_1, velocities=velocities, time_from_start=rospy.Duration(1.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_CLAP_POS_2, velocities=velocities, time_from_start=rospy.Duration(2.0)))
        l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_CLAP_POS_1, velocities=velocities, time_from_start=rospy.Duration(3.0)))
	l_traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.L_CLAP_POS_2, velocities=velocities, time_from_start=rospy.Duration(4.0)))
        l_traj_goal.trajectory.joint_names = self.l_joint_names
        # Set goal
	self.r_traj_action_client.send_goal(r_traj_goal)
	self.l_traj_action_client.send_goal(l_traj_goal)

    def relax_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = []
        stop_controllers = [controller_name]
        self.set_arm_mode(start_controllers, stop_controllers)

    def freeze_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = [controller_name]
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def set_arm_mode(self, start_controllers, stop_controllers):
        try:
            self.switch_service_client(start_controllers, stop_controllers, 1)
        except rospy.ServiceException:
            rospy.logerr('Could not change arm mode.')

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
            joint_states message is received'''
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.joint_sig.emit(msg)
        self.lock.release()

    def joint_sig_cb(self, msg):
	pass

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
    
    # Triggered when button is pressed: Place response actions here
    def command_cb(self):
        button_name = self._widget.sender().text()
	speechLabel = '';
        if (button_name == 'Speak'):
            qWarning('Robot will say: ' + self.text_in.text())
            self.show_text_in_rviz('Robot will say: ' + self.text_in.text())
            self._sound_client.say(self.text_in.text())
	    speechLabel = 'said: ' + self.text_in.text();
        elif (button_name == 'Head ^'):
            self.show_text_in_rviz('Robot Head Will Move Up')
            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = 'base_link'
            head_goal.min_duration = rospy.Duration(1.0)
            self.HEAD_Z += 0.1
            head_goal.target.point = Point(math.cos(self.HEAD_THETA), math.sin(self.HEAD_THETA), self.HEAD_Z)
            
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(10.0))
            if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
 	    speechLabel = 'Head is moving up';       
	elif (button_name == 'Head v'):
            self.show_text_in_rviz('Robot Head Will Move Down')
            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = 'base_link'
            head_goal.min_duration = rospy.Duration(1.0)
            self.HEAD_Z -= 0.1
            head_goal.target.point = Point(math.cos(self.HEAD_THETA), math.sin(self.HEAD_THETA), self.HEAD_Z)
            
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(10.0))
            if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
	    speechLabel = 'Head is moving down';
        elif (button_name == 'Head >'):
            self.show_text_in_rviz('Robot Head Will Move Right')
            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = 'base_link'
            head_goal.min_duration = rospy.Duration(1.0)
            self.HEAD_THETA -= 0.1
            head_goal.target.point = Point(math.cos(self.HEAD_THETA), math.sin(self.HEAD_THETA), self.HEAD_Z)
            
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(10.0))
            if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
	    speechLabel = 'Head is turning to the right';
        elif (button_name == '< Head'):
            self.show_text_in_rviz('Robot Head Will Move Left')
            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = 'base_link'
            head_goal.min_duration = rospy.Duration(1.0)
            self.HEAD_THETA += 0.1
            head_goal.target.point = Point(math.cos(self.HEAD_THETA), math.sin(self.HEAD_THETA), self.HEAD_Z)
            
            self.head_client.send_goal(head_goal)
            self.head_client.wait_for_result(rospy.Duration(10.0))
            if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
	    speechLabel = 'Head is turning to the left';
        elif (button_name == 'LGr Open'):
            self.show_text_in_rviz('Left Robot Gripper Will Open')
            gripper_goal = GripperCommandGoal()
            self.LEFT_GRIPPER_POSITION += self.GRIPPER_INTENSITY
            gripper_goal.command.position = self.LEFT_GRIPPER_POSITION
            gripper_goal.command.max_effort = 30.0
            
            self.l_gripper_client.send_goal(gripper_goal)
            self.l_gripper_client.wait_for_result(rospy.Duration(10.0))
            if (self.l_gripper_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Gripper action unsuccessful.')
	    speechLabel = 'Opening the left gripper';
        elif (button_name == 'LGr Close'):
            self.show_text_in_rviz('Left Robot Gripper Will Close')
            gripper_goal = GripperCommandGoal()
            self.LEFT_GRIPPER_POSITION -= self.GRIPPER_INTENSITY
            gripper_goal.command.position = self.LEFT_GRIPPER_POSITION
            gripper_goal.command.max_effort = 30.0
            
            self.l_gripper_client.send_goal(gripper_goal)
            self.l_gripper_client.wait_for_result(rospy.Duration(10.0))
            if (self.l_gripper_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Gripper action unsuccessful.')
	    speechLabel = 'Closing the left gripper';
        elif (button_name == 'RGr Open'):
            self.show_text_in_rviz('Right Robot Gripper Will Open')
            gripper_goal = GripperCommandGoal()
            self.RIGHT_GRIPPER_POSITION += self.GRIPPER_INTENSITY
            gripper_goal.command.position = self.RIGHT_GRIPPER_POSITION
            gripper_goal.command.max_effort = 30.0
            
            self.r_gripper_client.send_goal(gripper_goal)
            self.r_gripper_client.wait_for_result(rospy.Duration(10.0))
            if (self.r_gripper_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Gripper action unsuccessful.')
	    speechLabel = 'Opening the right gripper';
        elif (button_name == 'RGr Close'):
            self.show_text_in_rviz('Right Robot Gripper Will Close')
            gripper_goal = GripperCommandGoal()
            self.RIGHT_GRIPPER_POSITION -= self.GRIPPER_INTENSITY
            gripper_goal.command.position = self.RIGHT_GRIPPER_POSITION
            gripper_goal.command.max_effort = 30.0
            
            self.r_gripper_client.send_goal(gripper_goal)
            self.r_gripper_client.wait_for_result(rospy.Duration(10.0))
            if (self.r_gripper_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Gripper action unsuccessful.')
	    speechLabel = 'Closing the right gripper';
	elif (button_name == 'Twist >'):
            self.show_text_in_rviz('Robot base will twist right.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(self.LINEAR_X, self.LINEAR_Y, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, -1.0)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Twisting the robot base to the right';
        elif (button_name == '< Twist'):
            self.show_text_in_rviz('Robot base will twist left.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(self.LINEAR_X, self.LINEAR_Y, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, 1.0)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Twisting the robot base to the left';
	elif (button_name == 'Move ^'):
            self.show_text_in_rviz('Robot base will move forward.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(self.MOTION_INTENSITY, self.LINEAR_Y, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, self.ANGULAR_Z)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Move the robot forward';
        elif (button_name == 'Move v'):
            self.show_text_in_rviz('Robot base will move backward.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(-self.MOTION_INTENSITY, self.LINEAR_Y, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, self.ANGULAR_Z)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Move the robot backwards';
	elif (button_name == '< Move'):
            self.show_text_in_rviz('Robot base will move to the left.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(self.LINEAR_X, self.MOTION_INTENSITY, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, self.ANGULAR_Z)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Move the robot to the left';
        elif (button_name == 'Move >'):
            self.show_text_in_rviz('Robot base will move to the right.')
            twist_msg = Twist()
            twist_msg.linear = Vector3(self.LINEAR_X, -self.MOTION_INTENSITY, self.LINEAR_Z)
            twist_msg.angular = Vector3(self.ANGULAR_X, self.ANGULAR_Y, self.ANGULAR_Z)
            for i in range(100):
                self.base_pub.publish(twist_msg)
	    speechLabel = 'Move the robot to the right';
        elif (button_name == 'Relax right arm'):
            self.relax_arm('r')
            speechLabel = 'Relaxing right arm';
        elif (button_name == 'Freeze right arm'):
            self.freeze_arm('r')
            speechLabel = 'Freezing right arm';
        elif (button_name == 'Relax left arm'):
            self.relax_arm('l')
            speechLabel = 'Relaxing left arm';
        elif (button_name == 'Freeze left arm'):
            self.freeze_arm('l')
            speechLabel = 'Freezing left arm';
        elif (button_name == 'Save right arm pose'):
            self.save_pose('r')
            speechLabel = 'Saving right arm pose';
        elif (button_name == 'Save left arm pose'):
            self.save_pose('l')
            speechLabel = 'Saving left arm pose';
        elif (button_name == 'Move right arm to selected pose'):
            self.move_arm('r')
            speechLabel = 'Moving right arm';
        elif (button_name == 'Move left arm to selected pose'):
            self.move_arm('l')
            speechLabel = 'Moving left arm';
        elif (button_name == 'Delete selected right arm pose'):
            self.delete_pose('r')
            speechLabel = 'Deleting selected right arm pose';
        elif (button_name == 'Delete selected left arm pose'):
            self.delete_pose('l')
            speechLabel = 'Deleting selected left arm pose';
        elif (button_name == 'LShldr Pan Out'):
            positions = self.get_joint_state('l')
            positions[0] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Moving left shoulder outward';
        elif (button_name == 'LShldr Pan In'):
            positions = self.get_joint_state('l')
            positions[0] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Moving left shoulder inward';
        elif (button_name == 'LShldr Lift ^'):
            positions = self.get_joint_state('l')
            positions[1] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left shoulder upward';
        elif (button_name == 'LShldr Lift v'):
            positions = self.get_joint_state('l')
            positions[1] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left shoulder downward';
        elif (button_name == 'LUpArm Roll Out'):
            positions = self.get_joint_state('l')
            positions[2] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left upper arm outward';
        elif (button_name == 'LUpArm Roll In'):
            positions = self.get_joint_state('l')
            positions[2] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left upper arm inward';
        elif (button_name == 'LElbo Flex ^'):
            positions = self.get_joint_state('l')
            positions[3] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left elbow upward';
        elif (button_name == 'LElbo Flex v'):
            positions = self.get_joint_state('l')
            positions[3] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left elbow downward';
        elif (button_name == 'LLoArm Roll Out'):
            positions = self.get_joint_state('l')
            positions[4] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left lower arm outward';
        elif (button_name == 'LLoArm Roll In'):
            positions = self.get_joint_state('l')
            positions[4] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left lower arm inward';
        elif (button_name == 'LWrst Flex ^'):
            positions = self.get_joint_state('l')
            positions[5] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left wrist upward';
        elif (button_name == 'LWrst Flex v'):
            positions = self.get_joint_state('l')
            positions[5] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Flexing left wrist downward';
        elif (button_name == 'LWrst Roll Out'):
            positions = self.get_joint_state('l')
            positions[6] -= self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left wrist outward';
        elif (button_name == 'LWrst Roll In'):
            positions = self.get_joint_state('l')
            positions[6] += self.ARM_INTENSITY
            self.move_to_joints('l', positions, 0.0)
            speechLabel = 'Rolling left wrist inward';
        elif (button_name == 'RShldr Pan Out'):
            positions = self.get_joint_state('r')
            positions[0] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Moving right shoulder outward';
        elif (button_name == 'RShldr Pan In'):
            positions = self.get_joint_state('r')
            positions[0] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Moving right shoulder inward';
        elif (button_name == 'RShldr Lift ^'):
            positions = self.get_joint_state('r')
            positions[1] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right shoulder upward';
        elif (button_name == 'RShldr Lift v'):
            positions = self.get_joint_state('r')
            positions[1] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right shoulder downward';
        elif (button_name == 'RUpArm Roll Out'):
            positions = self.get_joint_state('r')
            positions[2] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right upper arm outward';
        elif (button_name == 'RUpArm Roll In'):
            positions = self.get_joint_state('r')
            positions[2] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right upper arm inward';
        elif (button_name == 'RElbo Flex ^'):
            positions = self.get_joint_state('r')
            positions[3] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right elbow upward';
        elif (button_name == 'RElbo Flex v'):
            positions = self.get_joint_state('r')
            positions[3] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right elbow downward';
        elif (button_name == 'RLoArm Roll Out'):
            positions = self.get_joint_state('r')
            positions[4] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right lower arm outward';
        elif (button_name == 'RLoArm Roll In'):
            positions = self.get_joint_state('r')
            positions[4] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right lower arm inward';
        elif (button_name == 'RWrst Flex ^'):
            positions = self.get_joint_state('r')
            positions[5] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right wrist upward';
        elif (button_name == 'RWrst Flex v'):
            positions = self.get_joint_state('r')
            positions[5] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Flexing right wrist downward';
        elif (button_name == 'RWrst Roll Out'):
            positions = self.get_joint_state('r')
            positions[6] += self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right wrist outward';
        elif (button_name == 'RWrst Roll In'):
            positions = self.get_joint_state('r')
            positions[6] -= self.ARM_INTENSITY
            self.move_to_joints('r', positions, 0.0)
            speechLabel = 'Rolling right wrist inward';
        elif (button_name == 'Wave Arm'):
            self.wave_motion()
            speechLabel = 'waving arm';
        elif (button_name == 'Clap Hands'):
            self.clap_motion()
            speechLabel = 'clapping hands';
        elif (button_name == 'Create arm movement'):
            self.create_motion()
            speechLabel = 'is creating new arm movement';
        elif (button_name == 'Initiate selected arm movement'):
            speechLabel = 'is moving its arms';
        elif (button_name == 'Delete arm movement'):
            speechLabel = 'is deleting selected arm movement';
        self.speech_label.setText('Robot ' + speechLabel)

    def create_motion(self):
        pop = MyPopup()
        pop._widget.show()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # Leave both arm controllers on
        start_controllers = ['r_arm_controller', 'l_arm_controller']
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

class MyPopup:
    _widget = QWidget(None)
    _widget.setWindowTitle('Create an arm movement')
    _widget.setObjectName('ArmMoveGUI')
    
    def __init__(self):
        # Main layout element for GUI
        large_box = QtGui.QVBoxLayout()
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        
        # Create Button box for GUI
        # addItem(item, int row, int column, int rowSpan = 1, int columnSpan = 1, Qt::Alignment alignment = 0)
	primary_box = QtGui.QGridLayout()
        # Add The Widgets to the grid
        self.name_label = QtGui.QLabel('Movement Name: ')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 1, 1, 1, 2)
        self.name_in = QtGui.QLineEdit('Default')
        primary_box.addWidget(self.name_in, 1, 3, 1, 8)

        self.name_label = QtGui.QLabel('Left Arm Positions')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 1, 1, 3)

        self.name_label = QtGui.QLabel('Start Time')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 4, 1, 1)

        self.name_label = QtGui.QLabel('Right Arm Positions')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 6, 1, 3)

        self.name_label = QtGui.QLabel('Start Time')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 9, 1, 1)

        self.l_arm_pose_box = QtGui.QComboBox()
        for i in range(0, SimpleGUI.l_arm_pose_box.count()):
            self.l_arm_pose_box.addItem(SimpleGUI.l_arm_pose_box.itemText(i))
        primary_box.addWidget(self.l_arm_pose_box, 3, 1, 1, 3)

        self.l_time_in = QtGui.QLineEdit('1.0')
        primary_box.addWidget(self.l_time_in, 3, 4, 1, 1)

        self.l_add_btn = QtGui.QPushButton('Add')
        self.l_add_btn.clicked.connect(self.add_l_pose)
        primary_box.addWidget(self.l_add_btn, 3, 5, 1, 1)

        self.r_arm_pose_box = QtGui.QComboBox()
        for i in range(0, SimpleGUI.r_arm_pose_box.count()):
            self.r_arm_pose_box.addItem(SimpleGUI.r_arm_pose_box.itemText(i))
        primary_box.addWidget(self.r_arm_pose_box, 3, 6, 1, 3)

        self.r_time_in = QtGui.QLineEdit('1.0')
        primary_box.addWidget(self.r_time_in, 3, 9, 1, 1)

        primary_box.addWidget(QtGui.QPushButton('Add'), 3, 10, 1, 1)

        self.l_text_box = QtGui.QPlainTextEdit()
        self.l_text_box.setReadOnly(True)
        primary_box.addWidget(self.l_text_box, 4, 1, 1, 5)

        self.r_text_box = QtGui.QPlainTextEdit()
        self.r_text_box.setReadOnly(True)
        primary_box.addWidget(self.r_text_box, 4, 6, 1, 5)

        primary_box.addWidget(QtGui.QPushButton('Save'), 5, 5, 1, 1)
        primary_box.addWidget(QtGui.QPushButton('Exit'), 5, 6, 1, 1)
        large_box.addLayout(primary_box)
        large_box.addItem(QtGui.QSpacerItem(100,20))
        
        # Puts layout in widget and adds widget name for RosGUI to identify
        self._widget.setLayout(large_box)
    
    # Will create a button, with all buttons using same event trigger
    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        #btn.clicked.connect(self.command_cb)
        return btn

    def add_l_pose(self):
        self.l_text_box.appendPlainText('Test')
        self.l_text_box.repaint()
        


