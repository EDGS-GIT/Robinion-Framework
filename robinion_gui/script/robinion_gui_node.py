#!/usr/bin/env python

import math as m
import numpy as np
import rospy
import actionlib
from PySide2 import QtWidgets
from PySide2.QtCore import QTimer
import robinion_gui as rb_gui
from geometry_msgs.msg import Pose
from robinion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionFeedback, PlayMotionResult
from robinion_msgs.msg import MoveArmAction, MoveArmGoal, MoveArmFeedback, MoveArmResult
from robinion_msgs.msg import WalkingState, WalkingCommand
from robinion_msgs.srv import CalcFK, CalcIK
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RobinionQtGUI(rb_gui.Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self, node_name):
        super(RobinionQtGUI, self).__init__()
        # Setup ROS Node
        rospy.init_node(node_name)
        self.setupUi(self)
        self.horizontalSliderCmdX.setTickInterval(1)
        self.horizontalSliderCmdX.setMinimum(-7)
        self.horizontalSliderCmdX.setMaximum(7)
        self.horizontalSliderCmdX.setValue(0)
        self.horizontalSliderCmdY.setTickInterval(1)
        self.horizontalSliderCmdY.setMinimum(-2)
        self.horizontalSliderCmdY.setMaximum(2)
        self.horizontalSliderCmdY.setValue(0)
        self.horizontalSliderCmdA.setTickInterval(1)
        self.horizontalSliderCmdA.setMinimum(-34)
        self.horizontalSliderCmdA.setMaximum(34)
        self.horizontalSliderCmdA.setValue(0)



        # ROS Comm Initialization
        self.rb_joint_states = JointStatesSubscriber()
        self.rb_joint_command = JointStatesPublisher()
        self.init_subscriber()
        self.init_publisher()
        self.init_service_server()
        self.init_service_client()
        self.init_action_server()
        self.init_action_client()

        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_alpha = 0

        # Tab gait controller
        self.pushButtonZeroPose.clicked.connect(self.call_zero_pose)
        self.pushButtonInitialize.clicked.connect(self.call_initialize_pose)
        self.pushButtonStanding.clicked.connect(self.call_standing_pose)
        self.pushButtonStartWalk.clicked.connect(self.call_start_walk)
        self.pushButtonStopWalk.clicked.connect(self.call_stop_walk)

        self.pushButtonForward.clicked.connect(self.call_walk_forward)
        self.pushButtonBackward.clicked.connect(self.call_walk_backward)
        self.pushButtonSideLeft.clicked.connect(self.call_walk_side_left)
        self.pushButtonSideRight.clicked.connect(self.call_walk_side_right)
        self.pushButtonTurnLeft.clicked.connect(self.call_walk_turn_left)
        self.pushButtonTurnRight.clicked.connect(self.call_walk_turn_right)

        self.horizontalSliderCmdX.valueChanged.connect(self.update_walk_command)
        self.horizontalSliderCmdY.valueChanged.connect(self.update_walk_command)
        self.horizontalSliderCmdA.valueChanged.connect(self.update_walk_command)

        # Tab gait parameter
        self.pushButtonLoadParameter.clicked.connect(self.load_parameters)
        self.pushButtonSaveParameter.clicked.connect(self.save_parameters)

        # Tab manipulation module
        self.pushButtonGetLHandPose.clicked.connect(self.update_current_left_hand_pose)
        self.pushButtonGetRHandPose.clicked.connect(self.update_current_right_hand_pose)
        self.pushButtonSetLHandPose.clicked.connect(self.send_left_hand_pose)
        self.pushButtonSetRHandPose.clicked.connect(self.send_right_hand_pose)

        # Tab head
        self.hS_HeadYawCommand.valueChanged.connect(self.send_head_yaw_command)
        self.hS_HeadPitchCommand.valueChanged.connect(self.send_head_pitch_command)

        # Tab arm
        self.hS_LShoulderPitchCommand.valueChanged.connect(self.send_l_shoulder_pitch_command)
        self.hS_LShoulderRollCommand.valueChanged.connect(self.send_l_shoulder_roll_command)
        self.hS_LElbowPitchCommand.valueChanged.connect(self.send_l_elbow_pitch_command)
        self.hS_LElbowYawCommand.valueChanged.connect(self.send_l_elbow_yaw_command)

        self.hS_RShoulderPitchCommand.valueChanged.connect(self.send_r_shoulder_pitch_command)
        self.hS_RShoulderRollCommand.valueChanged.connect(self.send_r_shoulder_roll_command)
        self.hS_RElbowPitchCommand.valueChanged.connect(self.send_r_elbow_pitch_command)
        self.hS_RElbowYawCommand.valueChanged.connect(self.send_r_elbow_yaw_command)

        # Tab leg
        self.hS_LHipYawCommand.valueChanged.connect(self.send_l_hip_yaw_command)
        self.hS_LHipRollCommand.valueChanged.connect(self.send_l_hip_roll_command)
        self.hS_LHipPitchCommand.valueChanged.connect(self.send_l_hip_pitch_command)
        self.hS_LAnklePitchCommand.valueChanged.connect(self.send_l_ankle_pitch_command)
        self.hS_LAnkleRollCommand.valueChanged.connect(self.send_l_ankle_roll_command)

        self.hS_RHipYawCommand.valueChanged.connect(self.send_r_hip_yaw_command)
        self.hS_RHipRollCommand.valueChanged.connect(self.send_r_hip_roll_command)
        self.hS_RHipPitchCommand.valueChanged.connect(self.send_r_hip_pitch_command)
        self.hS_RAnklePitchCommand.valueChanged.connect(self.send_r_ankle_pitch_command)
        self.hS_RAnkleRollCommand.valueChanged.connect(self.send_r_ankle_roll_command)

        # Load default paramaters
        self.load_parameters()
        self.initialize_ui_with_curent_states()

        # Timer for update joint states data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_joint_states_data)
        self.timer.start(10)

    def init_subscriber(self):
        pass
    
    def init_publisher(self):
        self.walking_state_pub = rospy.Publisher("/robinion/walking_module/state", WalkingState, queue_size=1)
        self.walking_command_pub = rospy.Publisher("/robinion/walking_module/command", WalkingCommand, queue_size=1)

    def init_service_server(self):
        pass

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_ik")
        self.calc_l_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_fk", CalcFK)
        self.calc_r_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_fk", CalcFK)
    
    def init_action_server(self):
        pass

    def init_action_client(self):
        self.play_motion_ac = actionlib.SimpleActionClient("/robinion/motion_player/play_motion", PlayMotionAction)
        self.play_motion_ac.wait_for_server()
        self.move_l_arm_ac = actionlib.SimpleActionClient("/robinion/manipulation_module/move_l_arm", MoveArmAction)
        self.move_r_arm_ac = actionlib.SimpleActionClient("/robinion/manipulation_module/move_r_arm", MoveArmAction)
        self.move_l_arm_ac.wait_for_server()
        self.move_r_arm_ac.wait_for_server()
    
    def initialize_ui_with_curent_states(self):
        # Tab head
        self.hS_HeadYawCommand.setValue(int(self.rb_joint_states.q_head[0]*100))
        self.hS_HeadPitchCommand.setValue(int(self.rb_joint_states.q_head[1]*100))

        # Tab arm
        self.hS_LShoulderPitchCommand.setValue(int(self.rb_joint_states.q_l_arm[0]*100))
        self.hS_LShoulderRollCommand.setValue(int(self.rb_joint_states.q_l_arm[1]*100))
        self.hS_LElbowPitchCommand.setValue(int(self.rb_joint_states.q_l_arm[2]*100))
        self.hS_LElbowYawCommand.setValue(int(self.rb_joint_states.q_l_arm[3]*100))

        self.hS_RShoulderPitchCommand.setValue(int(self.rb_joint_states.q_r_arm[0]*100))
        self.hS_RShoulderRollCommand.setValue(int(self.rb_joint_states.q_r_arm[1]*100))
        self.hS_RElbowPitchCommand.setValue(int(self.rb_joint_states.q_r_arm[2]*100))
        self.hS_RElbowYawCommand.setValue(int(self.rb_joint_states.q_r_arm[3]*100))

        # Tab leg
        self.hS_LHipYawCommand.setValue(int(self.rb_joint_states.q_l_leg[0]*100))
        self.hS_LHipRollCommand.setValue(int(self.rb_joint_states.q_l_leg[1]*100))
        self.hS_LHipPitchCommand.setValue(int(self.rb_joint_states.q_l_leg[2]*100))
        self.hS_LAnklePitchCommand.setValue(int(self.rb_joint_states.q_l_leg[3]*100))
        self.hS_LAnkleRollCommand.setValue(int(self.rb_joint_states.q_l_leg[4]*100))

        self.hS_RHipYawCommand.setValue(int(self.rb_joint_states.q_r_leg[0]*100))
        self.hS_RHipRollCommand.setValue(int(self.rb_joint_states.q_r_leg[1]*100))
        self.hS_RHipPitchCommand.setValue(int(self.rb_joint_states.q_r_leg[2]*100))
        self.hS_RAnklePitchCommand.setValue(int(self.rb_joint_states.q_r_leg[3]*100))
        self.hS_RAnkleRollCommand.setValue(int(self.rb_joint_states.q_r_leg[4]*100))
    
    def update_joint_states_data(self):
        self.labelHeadYawCurrent.setText("{:.3f}".format(self.rb_joint_states.q_head[0]))
        self.labelHeadPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_head[1]))

        self.labelLShoulderPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_arm[0]))
        self.labelLShoulderRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_arm[1]))
        self.labelLElbowPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_arm[2]))
        self.labelLElbowYawCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_arm[3]))

        self.labelRShoulderPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_arm[0]))
        self.labelRShoulderRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_arm[1]))
        self.labelRElbowPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_arm[2]))
        self.labelRElbowYawCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_arm[3]))

        self.labelLHipYawCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_leg[0]))
        self.labelLHipRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_leg[1]))
        self.labelLHipPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_leg[2]))
        self.labelLAnklePitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_leg[3]))
        self.labelLAnkleRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_l_leg[4]))

        self.labelRHipYawCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_leg[0]))
        self.labelRHipRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_leg[1]))
        self.labelRHipPitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_leg[2]))
        self.labelRAnklePitchCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_leg[3]))
        self.labelRAnkleRollCurrent.setText("{:.3f}".format(self.rb_joint_states.q_r_leg[4]))
    
    def call_zero_pose(self):
        motion_data = PlayMotionGoal()
        motion_data.motion_name = "zero"
        motion_data.movement_time = 2.0
        self.play_motion_ac.send_goal(motion_data)
        self.play_motion_ac.wait_for_result()
        success = self.play_motion_ac.get_result()
        rospy.loginfo("Call Zero Pose : %s", success)
    
    def call_initialize_pose(self):
        motion_data = PlayMotionGoal()
        motion_data.motion_name = "initialize"
        motion_data.movement_time = 2.0
        self.play_motion_ac.send_goal(motion_data)
        self.play_motion_ac.wait_for_result()
        success = self.play_motion_ac.get_result()
        rospy.loginfo("Call Initialize Pose : %s", success)
    
    def call_standing_pose(self):
        motion_data = PlayMotionGoal()
        motion_data.motion_name = "standing"
        motion_data.movement_time = 2.0
        self.play_motion_ac.send_goal(motion_data)
        self.play_motion_ac.wait_for_result()
        success = self.play_motion_ac.get_result()
        rospy.loginfo("Call Standing Pose : %s", success)
    
    def call_start_walk(self):
        ws = WalkingState()
        ws.state = "start"
        self.walking_state_pub.publish(ws)
        rospy.loginfo("Call Start Walk")

    def call_stop_walk(self):
        ws = WalkingState()
        ws.state = "stop"
        self.walking_state_pub.publish(ws)
        rospy.loginfo("Call Stop Walk")
    
    def call_walk_forward(self):
        self.cmd_x = 0.03
        self.cmd_y = 0.00
        self.cmd_alpha = 0.00
        self.update_slider_value()
        self.send_walk_command()
        rospy.loginfo("Walk Forward")
    
    def call_walk_backward(self):
        rospy.loginfo("Walk Backward")
    
    def call_walk_side_left(self):
        self.cmd_x = 0.00
        self.cmd_y = 0.01
        self.cmd_alpha = 0.00
        self.update_slider_value()
        self.send_walk_command()
        rospy.loginfo("Walk Side Left")
    
    def call_walk_side_right(self):
        self.cmd_x = 0.00
        self.cmd_y = -0.01
        self.cmd_alpha = 0.00
        self.update_slider_value()
        self.send_walk_command()
        rospy.loginfo("Walk Side Right")
    
    def call_walk_turn_left(self):
        self.cmd_x = 0.00
        self.cmd_y = 0.00
        self.cmd_alpha = m.radians(10)
        self.update_slider_value()
        self.send_walk_command()
        rospy.loginfo("Walk Turn Left")
    
    def call_walk_turn_right(self):
        self.cmd_x = 0.00
        self.cmd_y = 0.00
        self.cmd_alpha = m.radians(-10)
        self.update_slider_value()
        self.send_walk_command()
        rospy.loginfo("Walk Turn Right")
    
    def update_walk_command(self):
        self.cmd_x = float(self.horizontalSliderCmdX.value() / 100.0)
        self.cmd_y = float(self.horizontalSliderCmdY.value() / 100.0)
        self.cmd_alpha = float(self.horizontalSliderCmdA.value() / 100.0)
        self.send_walk_command()

    def send_walk_command(self):
        wc = WalkingCommand()
        wc.cmd_x = self.cmd_x
        wc.cmd_y = self.cmd_y
        wc.cmd_alpha = self.cmd_alpha
        self.walking_command_pub.publish(wc)
    
    def update_slider_value(self):
        self.horizontalSliderCmdX.setValue(int(self.cmd_x*100))
        self.horizontalSliderCmdY.setValue(int(self.cmd_y*100))
        self.horizontalSliderCmdA.setValue(int(self.cmd_alpha*100))

    def update_current_left_hand_pose(self):
        q_l_hand_with_torso_current = self.rb_joint_states.q_l_arm_with_torso.copy()
        result = self.calc_l_arm_fk_srv(q_l_hand_with_torso_current)
        rospy.loginfo("Current Pose : %s", result.pose)
        x = result.pose.position.x
        y = result.pose.position.y
        z = result.pose.position.z
        q_list = [result.pose.orientation.x, result.pose.orientation.y,
                  result.pose.orientation.y, result.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.lineEditCurrentLHandX.setText("{:.3f}".format(x))
        self.lineEditCurrentLHandY.setText("{:.3f}".format(y))
        self.lineEditCurrentLHandZ.setText("{:.3f}".format(z))
        self.lineEditCurrentLHandRoll.setText("{:.3f}".format(m.degrees(roll)))
        self.lineEditCurrentLHandPitch.setText("{:.3f}".format(m.degrees(pitch)))
        self.lineEditCurrentLHandYaw.setText("{:.3f}".format(m.degrees(yaw)))
        # Line edit
        self.lineEditSetLHandX.setText("{:.3f}".format(x))
        self.lineEditSetLHandY.setText("{:.3f}".format(y))
        self.lineEditSetLHandZ.setText("{:.3f}".format(z))
        self.lineEditSetLHandRoll.setText("{:.3f}".format(m.degrees(roll)))
        self.lineEditSetLHandPitch.setText("{:.3f}".format(m.degrees(pitch)))
        self.lineEditSetLHandYaw.setText("{:.3f}".format(m.degrees(yaw)))
    
    def update_current_right_hand_pose(self):
        q_r_hand_with_torso_current = self.rb_joint_states.q_r_arm_with_torso.copy()
        result = self.calc_r_arm_fk_srv(q_r_hand_with_torso_current)
        x = result.pose.position.x
        y = result.pose.position.y
        z = result.pose.position.z
        q_list = [result.pose.orientation.x, result.pose.orientation.y,
                  result.pose.orientation.y, result.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.lineEditCurrentRHandX.setText("{:.3f}".format(x))
        self.lineEditCurrentRHandY.setText("{:.3f}".format(y))
        self.lineEditCurrentRHandZ.setText("{:.3f}".format(z))
        self.lineEditCurrentRHandRoll.setText("{:.3f}".format(m.degrees(roll)))
        self.lineEditCurrentRHandPitch.setText("{:.3f}".format(m.degrees(pitch)))
        self.lineEditCurrentRHandYaw.setText("{:.3f}".format(m.degrees(yaw)))
        # Line edit
        self.lineEditSetRHandX.setText("{:.3f}".format(x))
        self.lineEditSetRHandY.setText("{:.3f}".format(y))
        self.lineEditSetRHandZ.setText("{:.3f}".format(z))
        self.lineEditSetRHandRoll.setText("{:.3f}".format(m.degrees(roll)))
        self.lineEditSetRHandPitch.setText("{:.3f}".format(m.degrees(pitch)))
        self.lineEditSetRHandYaw.setText("{:.3f}".format(m.degrees(yaw)))
    
    def send_left_hand_pose(self):
        # rospy.loginfo("Send Left Hand Pose")
        goal_pose = Pose()
        goal_pose.position.x = float(self.lineEditSetLHandX.text())
        goal_pose.position.y = float(self.lineEditSetLHandY.text())
        goal_pose.position.z = float(self.lineEditSetLHandZ.text())
        roll = m.radians(float(self.lineEditSetLHandRoll.text()))
        pitch = m.radians(float(self.lineEditSetLHandPitch.texrb_joint_commandt()))
        yaw = m.radians(float(self.lineEditSetLHandYaw.text()))
        q = quaternion_from_euler (roll, pitch,yaw)
        goal_pose.orientation.x = q[0]
        goal_pose.orientation.y = q[1]
        goal_pose.orientation.z = q[2]
        goal_pose.orientation.w = q[3]
        # rospy.loginfo("Goal Pose : %s", goal_pose)
        l_arm_data = MoveArmGoal()
        l_arm_data.target_pose = goal_pose
        l_arm_data.movement_time = 2.0
        l_arm_data.dt = 0.01
        self.move_l_arm_ac.send_goal(l_arm_data)
        self.move_l_arm_ac.wait_for_result()
        success = self.move_l_arm_ac.get_result()
        rospy.loginfo("%s", success)
    
    def send_right_hand_pose(self):
        # rospy.loginfo("Send Left Hand Pose")
        goal_pose = Pose()
        goal_pose.position.x = float(self.lineEditSetRHandX.text())
        goal_pose.position.y = float(self.lineEditSetRHandY.text())
        goal_pose.position.z = float(self.lineEditSetRHandZ.text())
        roll = m.radians(float(self.lineEditSetRHandRoll.text()))
        pitch = m.radians(float(self.lineEditSetRHandPitch.text()))
        yaw = m.radians(float(self.lineEditSetRHandYaw.text()))
        q = quaternion_from_euler (roll, pitch,yaw)
        goal_pose.orientation.x = q[0]
        goal_pose.orientation.y = q[1]
        goal_pose.orientation.z = q[2]
        goal_pose.orientation.w = q[3]
        # rospy.loginfo("Goal Pose : %s", goal_pose)
        r_arm_data = MoveArmGoal()
        r_arm_data.target_pose = goal_pose
        r_arm_data.movement_time = 2.0
        r_arm_data.dt = 0.01
        self.move_r_arm_ac.send_goal(r_arm_data)
        self.move_r_arm_ac.wait_for_result()
        success = self.move_r_arm_ac.get_result()
        rospy.loginfo("%s", success)
    
    def load_parameters(self):
        min_vx = rospy.get_param("/gait_param/limit/min_vx")
        max_vx = rospy.get_param("/gait_param/limit/max_vx")
        min_vy = rospy.get_param("/gait_param/limit/min_vy")
        max_vy = rospy.get_param("/gait_param/limit/max_vy")
        min_va = rospy.get_param("/gait_param/limit/min_va")
        max_va = rospy.get_param("/gait_param/limit/max_va")

        g_t_step = rospy.get_param("/gait_param/gazebo/t_step")
        g_dsp_ratio = rospy.get_param("/gait_param/gazebo/dsp_ratio")
        g_swing_foot_height = rospy.get_param("/gait_param/gazebo/swing_foot_height")
        g_support_x = rospy.get_param("/gait_param/gazebo/support_x")
        g_support_y = rospy.get_param("/gait_param/gazebo/support_y")
        g_body_tilt = rospy.get_param("/gait_param/gazebo/body_tilt")

        r_t_step = rospy.get_param("/gait_param/robot/t_step")
        r_dsp_ratio = rospy.get_param("/gait_param/robot/dsp_ratio")
        r_swing_foot_height = rospy.get_param("/gait_param/robot/swing_foot_height")
        r_support_x = rospy.get_param("/gait_param/robot/support_x")
        r_support_y = rospy.get_param("/gait_param/robot/support_y")
        r_body_tilt = rospy.get_param("/gait_param/robot/body_tilt")

        # update UI
        self.dSB_min_vx.setValue(min_vx)
        self.dSB_max_vx.setValue(max_vx)
        self.dSB_min_vy.setValue(min_vy)
        self.dSB_max_vy.setValue(max_vy)
        self.dSB_min_va.setValue(min_va)
        self.dSB_max_va.setValue(max_va)

        self.dSB_gz_t_step.setValue(g_t_step)
        self.dSB_gz_dsp_ratio.setValue(g_dsp_ratio)
        self.dSB_gz_swing_foot_height.setValue(g_swing_foot_height)
        self.dSB_gz_support_x.setValue(g_support_x)
        self.dSB_gz_support_y.setValue(g_support_y)
        self.dSB_gz_body_tilt.setValue(g_body_tilt)

        self.dSB_rb_t_step.setValue(r_t_step)
        self.dSB_rb_dsp_ratio.setValue(r_dsp_ratio)
        self.dSB_rb_swing_foot_height.setValue(r_swing_foot_height)
        self.dSB_rb_support_x.setValue(r_support_x)
        self.dSB_rb_support_y.setValue(r_support_y)
        self.dSB_rb_body_tilt.setValue(r_body_tilt)
        
    def save_parameters(self):
        rospy.set_param("/gait_param/limit/min_vx", self.dSB_min_vx.value())
        rospy.set_param("/gait_param/limit/max_vx", self.dSB_max_vx.value())
        rospy.set_param("/gait_param/limit/min_vy", self.dSB_min_vy.value())
        rospy.set_param("/gait_param/limit/max_vy", self.dSB_max_vy.value())
        rospy.set_param("/gait_param/limit/min_va", self.dSB_min_va.value())
        rospy.set_param("/gait_param/limit/max_va", self.dSB_max_va.value())

        rospy.set_param("/gait_param/gazebo/t_step", self.dSB_gz_t_step.value())
        rospy.set_param("/gait_param/gazebo/dsp_ratio", self.dSB_gz_dsp_ratio.value())
        rospy.set_param("/gait_param/gazebo/swing_foot_height", self.dSB_gz_swing_foot_height.value())
        rospy.set_param("/gait_param/gazebo/support_x", self.dSB_gz_support_x.value())
        rospy.set_param("/gait_param/gazebo/support_y", self.dSB_gz_support_y.value())
        rospy.set_param("/gait_param/gazebo/body_tilt", self.dSB_gz_body_tilt.value())

        rospy.set_param("/gait_param/robot/t_step", self.dSB_rb_t_step.value())
        rospy.set_param("/gait_param/robot/dsp_ratio", self.dSB_rb_dsp_ratio.value())
        rospy.set_param("/gait_param/robot/swing_foot_height", self.dSB_rb_swing_foot_height.value())
        rospy.set_param("/gait_param/robot/support_x", self.dSB_rb_support_x.value())
        rospy.set_param("/gait_param/robot/support_y", self.dSB_rb_support_y.value())
        rospy.set_param("/gait_param/robot/body_tilt", self.dSB_rb_body_tilt.value())

    def head_command(self):
        head = np.array([float (self.hS_HeadYawCommand.value() / 100.0),
                         float (self.hS_HeadPitchCommand.value() / 100.0)])
        self.rb_joint_command.set_head(head)

    def send_head_yaw_command(self):
        self.head_command()

    def send_head_pitch_command(self):
        self.head_command()

    def l_arm_command(self):
        l_arm = np.array([float (self.hS_LShoulderPitchCommand.value() / 100.0),
                          float (self.hS_LShoulderRollCommand.value() / 100.0),
                          float (self.hS_LElbowPitchCommand.value() / 100.0),
                          float (self.hS_LElbowYawCommand.value() / 100.0),])
        self.rb_joint_command.set_l_arm(l_arm)

    def send_l_shoulder_pitch_command(self):
        self.l_arm_command()

    def send_l_shoulder_roll_command(self):
        self.l_arm_command()

    def send_l_elbow_pitch_command(self):
        self.l_arm_command()

    def send_l_elbow_yaw_command(self):
        self.l_arm_command()
    
    def r_arm_command(self):
        r_arm = np.array([float (self.hS_RShoulderPitchCommand.value() / 100.0),
                          float (self.hS_RShoulderRollCommand.value() / 100.0),
                          float (self.hS_RElbowPitchCommand.value() / 100.0),
                          float (self.hS_RElbowYawCommand.value() / 100.0),])
        self.rb_joint_command.set_r_arm(r_arm)

    def send_r_shoulder_pitch_command(self):
        self.r_arm_command()

    def send_r_shoulder_roll_command(self):
        self.r_arm_command()

    def send_r_elbow_pitch_command(self):
        self.r_arm_command()

    def send_r_elbow_yaw_command(self):
        self.r_arm_command()

    def l_leg_command(self):
        l_leg = np.array([float (self.hS_LHipYawCommand.value() / 100.0),
                          float (self.hS_LHipRollCommand.value() / 100.0),
                          float (self.hS_LHipPitchCommand.value() / 100.0),
                          float (self.hS_LAnklePitchCommand.value() / 100.0),
                          float (self.hS_LAnkleRollCommand.value() / 100.0),])
        self.rb_joint_command.set_l_leg(l_leg)

    def send_l_hip_yaw_command(self):
        self.l_leg_command()

    def send_l_hip_roll_command(self):
        self.l_leg_command()

    def send_l_hip_pitch_command(self):
        self.l_leg_command()

    def send_l_ankle_pitch_command(self):
        self.l_leg_command()

    def send_l_ankle_roll_command(self):
        self.l_leg_command()
    
    def r_leg_command(self):
        r_leg = np.array([float (self.hS_RHipYawCommand.value() / 100.0),
                          float (self.hS_RHipRollCommand.value() / 100.0),
                          float (self.hS_RHipPitchCommand.value() / 100.0),
                          float (self.hS_RAnklePitchCommand.value() / 100.0),
                          float (self.hS_RAnkleRollCommand.value() / 100.0),])
        self.rb_joint_command.set_r_leg(r_leg)

    def send_r_hip_yaw_command(self):
        self.r_leg_command()

    def send_r_hip_roll_command(self):
        self.r_leg_command()

    def send_r_hip_pitch_command(self):
        self.r_leg_command()

    def send_r_ankle_pitch_command(self):
        self.r_leg_command()

    def send_r_ankle_roll_command(self):
        self.r_leg_command()

if __name__ == "__main__":
    node_name = "robinion_gui_node"
    rb_node = QtWidgets.QApplication()
    rb_node_gui = RobinionQtGUI(node_name)
    rb_node_gui.show()
    rb_node.exec_()