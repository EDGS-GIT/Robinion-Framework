#!/usr/bin/env python

import math as m
import rospy
import actionlib
from PySide2 import QtWidgets
import robinion_gui as rb_gui
from geometry_msgs.msg import Pose
from robinion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionFeedback, PlayMotionResult
from robinion_msgs.msg import MoveArmAction, MoveArmGoal, MoveArmFeedback, MoveArmResult
from robinion_msgs.msg import WalkingState, WalkingCommand
from robinion_msgs.srv import CalcFK, CalcIK
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
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
        self.rb_joint_states_data = JointStatesSubscriber()
        self.init_subscriber()
        self.init_publisher()
        self.init_service_server()
        self.init_service_client()
        self.init_action_server()
        self.init_action_client()

        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_alpha = 0

        # Signal and Slots
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

        self.pushButtonGetLHandPose.clicked.connect(self.update_current_left_hand_pose)
        self.pushButtonGetRHandPose.clicked.connect(self.update_current_right_hand_pose)
        # Slot to arm motion
        self.pushButtonSetLHandPose.clicked.connect(self.send_left_hand_pose)
        self.pushButtonSetRHandPose.clicked.connect(self.send_right_hand_pose)

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
        self.cmd_x = -0.03
        self.cmd_y = 0.00
        self.cmd_alpha = 0.00
        self.update_slider_value()
        self.send_walk_command()
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
        q_l_hand_with_torso_current = self.rb_joint_states_data.q_l_arm_with_torso.copy()
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
        q_r_hand_with_torso_current = self.rb_joint_states_data.q_r_arm_with_torso.copy()
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
        pitch = m.radians(float(self.lineEditSetLHandPitch.text()))
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

if __name__ == "__main__":
    node_name = "robinion_gui_node"
    rb_node = QtWidgets.QApplication()
    rb_node_gui = RobinionQtGUI(node_name)
    rb_node_gui.show()
    rb_node.exec_()