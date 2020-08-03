#!/usr/bin/env python

import os
import rospy
import actionlib
import numpy as np 
import math as m
import scipy.io 
import itertools
from enum import Enum
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from pytransform3d.rotations import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from robinion_msgs.msg import WalkingCommand, WalkingState, \
                              PlayMotionAction, PlayMotionGoal, PlayMotionFeedback, PlayMotionResult
from robinion_msgs.srv import CalcFK, CalcIK
from kinematics_pkg.kinematics import Kinematics
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher

class SupportFoot(Enum):
    RIGHT_SUPPORT = 0
    LEFT_SUPPORT = 1

class WalkingPhase(Enum):
    SSP_RIGHT = 0
    SSP_LEFT= 1
    DSP = 2

class GaitControllerState(Enum):
    INITIALIZE = 0
    START = 1
    STOP = 2

class FIFOBuffer(object):
    def __init__(self, len):
        super(FIFOBuffer, self).__init__()
        self.len = len 
        self.data = []

    def push(self, data):
        if len(self.data) >= self.len:
            self.data.pop(0)
        self.data.append(data)

    def get_first_value(self):
        return self.data[0]

    def get_last_value(self):
        return self.data[-1]

    def get_value_at_index(self, index):
        return self.data[index]

    def is_full(self):
        if len(self.data) >= self.len:
            return True
        else:
            return False

class GaitController():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        self.rb_kinematics = Kinematics()
        self.rb_joint_states_pub = JointStatesPublisher()
        self.state = GaitControllerState.STOP
        self.robot_standing = False
        self.command = ""
        self.load_param()
        self.load_timing_param()
        self.init_gait()
        # ROS Comm
        self.init_subscriber()
        self.init_publisher()
        self.init_service_client()
        self.init_service_server()
        self.init_action_client()
        self.init_action_server()

    def init_subscriber(self):
        rospy.Subscriber("/robinion/walking_module/state", WalkingState, self.state_callback)
        rospy.Subscriber("/robinion/walking_module/command", WalkingCommand, self.walking_command_callback)

    def init_publisher(self):
        self.gait_state_pub = rospy.Publisher('/robinion/walking_module/current_state', WalkingState, queue_size=10)

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_ik")
        self.calc_l_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_fk", CalcFK)
        self.calc_r_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_fk", CalcFK)
        self.calc_l_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_ik", CalcIK)
        self.calc_r_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_ik", CalcIK)

    def init_service_server(self):
        return

    def init_action_client(self):
        self.play_motion_ac = actionlib.SimpleActionClient("/robinion/motion_player/play_motion", PlayMotionAction)
        self.play_motion_ac.wait_for_server()
    
    def init_action_server(self):
        return

    # Callback Function
    def state_callback(self, data):
        self.command = data.state

    def walking_command_callback(self, data):
        self.cmd_x = data.cmd_x
        self.cmd_y = data.cmd_y 
        self.cmd_alpha = data.cmd_alpha

    # Core Method
    def load_param(self):
        rospy.loginfo("Load Param")
        self.simulation_mode = True
        # Hip offset
        self.hip_offset = self.rb_kinematics.HIP_TO_CROTCH
        # Limit
        self.cmd_x_min = rospy.get_param("/gait_param/limit/min_vx")
        self.cmd_x_max = rospy.get_param("/gait_param/limit/max_vx")
        self.cmd_y_min = rospy.get_param("/gait_param/limit/min_vy")
        self.cmd_y_max = rospy.get_param("/gait_param/limit/max_vy")
        self.cmd_alpha_min = m.radians(rospy.get_param("/gait_param/limit/min_va"))
        self.cmd_alpha_max = m.radians(rospy.get_param("/gait_param/limit/max_va"))
        if self.simulation_mode:
            self.support_x = rospy.get_param("/gait_param/gazebo/support_x")
            self.support_y = rospy.get_param("/gait_param/gazebo/support_y")
            self.body_tilt = m.radians(rospy.get_param("/gait_param/gazebo/body_tilt"))
            self.max_swing_height = rospy.get_param("/gait_param/gazebo/swing_foot_height")
        else:
            self.support_x = rospy.get_param("/gait_param/robot/support_x")
            self.support_y = rospy.get_param("/gait_param/robot/support_y")
            self.body_tilt = m.radians(rospy.get_param("/gait_param/robot/body_tilt"))
            self.max_swing_height = rospy.get_param("/gait_param/robot/swing_foot_height")

    def init_gait(self):
        # Motion Vector Input
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_alpha = 0.0
        
        # Step Length
        self.sx = 0.0
        self.sy = 0.0
        self.sa = 0.0
        
        # Initial footsteps data
        self.footsteps = FIFOBuffer(3)
        self.footsteps.push([0, -1 * (self.hip_offset + self.support_y), 0])
        self.footsteps.push([0, (self.hip_offset + self.support_y), 0])

        self.support_foot = SupportFoot.RIGHT_SUPPORT
        self.walking_phase = WalkingPhase.DSP

        self.stop_signal = False

        self.com_xyz_yaw = [0,0,0,0]
        self.com_yaw = 0.0

        # Initial and target heading of COM
        self.init_com_yaw = 0.0 
        self.goal_com_yaw = 0.0

        # Initial and target pose  of left and right sole 
        self.init_l_sole_xyz = np.zeros((3,1), dtype=float)
        self.init_l_sole_yaw = 0.0 
        self.goal_l_sole_xyz = np.zeros((3,1), dtype=float)
        self.goal_l_sole_yaw = 0.0
        self.init_r_sole_xyz = np.zeros((3,1), dtype=float)
        self.init_r_sole_yaw = 0.0
        self.goal_r_sole_xyz = np.zeros((3,1), dtype=float)
        self.goal_r_sole_yaw = 0.0      

        # x, y, z, yaw
        self.current_l_sole_xyz_yaw = [0, (self.hip_offset + self.support_y), 0, 0]
        self.current_r_sole_xyz_yaw = [0, -1*(self.hip_offset + self.support_y), 0, 0]

        self.t_bez = 0
        self.t = 0

    @staticmethod
    def rot_path(init_angle, target_angle, time, t):
        p0 = np.array([[0],[init_angle]])
        p1 = np.array([[0],[target_angle]])
        p2 = np.array([[time],[target_angle]])
        p3 = np.array([[time],[target_angle]])
        path = np.power((1-t), 3)*p0 + 3*np.power((1-t), 2)*t*p1 + 3*(1-t)*np.power(t, 2)*p2 + np.power(t, 3)*p3
        return path

    @staticmethod
    def swing_foot_path(str_pt, end_pt, swing_height, t):
        p0 = str_pt.copy()
        p1 = str_pt.copy()
        p1[2,0] = swing_height+(0.25*swing_height)
        p2 = end_pt.copy()
        p2[2,0] = swing_height+(0.25*swing_height)
        p3 = end_pt.copy()
        path = np.power((1-t), 3)*p0 + 3*np.power((1-t), 2)*t*p1 + 3*(1-t)*np.power(t, 2)*p2 + np.power(t, 3)*p3
        return path

    @staticmethod
    def create_tf_matrix(list_xyz_yaw):
        T_mat = np.eye(4)
        T_mat[0,3] = list_xyz_yaw[0]
        T_mat[1,3] = list_xyz_yaw[1]
        T_mat[2,3] = list_xyz_yaw[2]
        # Ini harus dalam format wxyz
        q = quaternion_from_euler(0, 0, list_xyz_yaw[3])
        R_mat = matrix_from_quaternion([q[3], q[0], q[1], q[2]])
        T_mat[:3,:3] = R_mat
        return T_mat

    @staticmethod
    def tf_matrix_from_xyz_rpy(list_xyz_rpy):
        T_mat = np.eye(4)
        T_mat[0,3] = list_xyz_rpy[0]
        T_mat[1,3] = list_xyz_rpy[1]
        T_mat[2,3] = list_xyz_rpy[2]
        # Ini harus dalam format wxyz
        q = quaternion_from_euler(list_xyz_rpy[3], list_xyz_rpy[4], list_xyz_rpy[5])
        R_mat = matrix_from_quaternion([q[3], q[0], q[1], q[2]])
        T_mat[:3,:3] = R_mat
        return T_mat

    def set_leg_pose(self, r_sole_pose, l_sole_pose):
        try:
            self.l_leg_srv(l_sole_pose[0], l_sole_pose[1], l_sole_pose[2], 
                           l_sole_pose[3], l_sole_pose[4], l_sole_pose[5], l_sole_pose[6], 
                           self.support_leg_torque)
            self.r_leg_srv(r_sole_pose[0], r_sole_pose[1], r_sole_pose[2], 
                           r_sole_pose[3], r_sole_pose[4], r_sole_pose[5], r_sole_pose[6],
                           self.support_leg_torque)
        except rospy.ServiceException as e:
            rospy.logwarn("Service Failed :" + str(e))

    # Here we define minimum and maximum value of motion vector
    def clip_motion_vector(self):
        if self.cmd_x < self.cmd_x_min:
            self.cmd_x = self.cmd_x_min
        if self.cmd_x > self.cmd_x_max:
            self.cmd_x = self.cmd_x_max
        if self.cmd_y < self.cmd_y_min:
            self.cmd_y = self.cmd_y_min
        if self.cmd_y > self.cmd_y_max:
            self.cmd_y = self.cmd_y_max
        if self.cmd_alpha < self.cmd_alpha_min:
            self.cmd_alpha = self.cmd_alpha_min
        if self.cmd_alpha > self.cmd_alpha_max:
            self.cmd_alpha = self.cmd_alpha_max

    def swap_support_foot(self):
        if self.support_foot == SupportFoot.RIGHT_SUPPORT:
            self.support_foot = SupportFoot.LEFT_SUPPORT
        else:
            self.support_foot = SupportFoot.RIGHT_SUPPORT
        
    def add_new_footstep(self):
        self.clip_motion_vector()
        self.swap_support_foot()
        if self.support_foot == SupportFoot.LEFT_SUPPORT: 
            self.sx = self.cmd_x
            self.sy = -(2*self.hip_offset + 2*self.support_y) + self.cmd_y 
            self.sa += self.cmd_alpha
            dx = self.footsteps.data[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
            dy = self.footsteps.data[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
            self.footsteps.push([dx, dy, self.sa])
        elif self.support_foot == SupportFoot.RIGHT_SUPPORT:
            self.sx = self.cmd_x 
            self.sy = (2*self.hip_offset + 2*self.support_y) + self.cmd_y
            self.sa += self.cmd_alpha
            dx = self.footsteps.data[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
            dy = self.footsteps.data[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
            self.footsteps.push([dx, dy, self.sa])

    def get_sole_trajectory(self):
        if self.t == 0:
            if self.support_foot == SupportFoot.LEFT_SUPPORT: # support kaki kiri
                # ambil vector posisi dari vector pose
                self.init_r_sole_xyz[0,0] = self.current_r_sole_xyz_yaw[0]
                self.init_r_sole_xyz[1,0] = self.current_r_sole_xyz_yaw[1]
                self.init_r_sole_xyz[2,0] = 0
                self.init_r_sole_yaw = self.current_r_sole_xyz_yaw[3]

                # ambil vector posisi 
                self.goal_r_sole_xyz[0,0] = self.footsteps.data[2][0]
                self.goal_r_sole_xyz[1,0] = self.footsteps.data[2][1]
                self.goal_r_sole_xyz[2,0] = 0
                self.goal_r_sole_yaw = self.footsteps.data[2][2]

                self.init_com_yaw = (float(self.current_l_sole_xyz_yaw[3]) + self.init_r_sole_yaw) / 2
                self.goal_com_yaw = (float(self.current_l_sole_xyz_yaw[3]) + self.goal_r_sole_yaw) / 2

            elif self.support_foot == SupportFoot.RIGHT_SUPPORT: # Support kaki kanan
                self.init_l_sole_xyz[0,0] = self.current_l_sole_xyz_yaw[0]
                self.init_l_sole_xyz[1,0] = self.current_l_sole_xyz_yaw[1]
                self.init_l_sole_xyz[2,0] = 0
                self.init_l_sole_yaw = self.current_l_sole_xyz_yaw[3]

                self.goal_l_sole_xyz[0,0] = self.footsteps.data[2][0]
                self.goal_l_sole_xyz[1,0] = self.footsteps.data[2][1]
                self.goal_l_sole_xyz[2,0] = 0
                self.goal_l_sole_yaw = self.footsteps.data[2][2]

                self.init_com_yaw = (float(self.current_r_sole_xyz_yaw[3]) + self.init_l_sole_yaw) / 2
                self.goal_com_yaw = (float(self.current_r_sole_xyz_yaw[3]) + self.goal_l_sole_yaw) / 2

        # Generate foot trajectory untuk kaki kanan dan kaki kiri
        # DSP Phase
        if self.t < (self.t_dsp/2.0) or self.t >= (self.t_dsp/2.0 + self.t_ssp):
            self.walking_phase = WalkingPhase.DSP
            self.t_bez = 0
        # SSP Phase
        else:
            if self.support_foot == SupportFoot.LEFT_SUPPORT:
                self.walking_phase = WalkingPhase.SSP_LEFT
                self.current_l_sole_xyz_yaw[0] = self.footsteps.data[1][0]
                self.current_l_sole_xyz_yaw[1] = self.footsteps.data[1][1]
                self.current_l_sole_xyz_yaw[2] = 0
                self.current_l_sole_xyz_yaw[3] = self.footsteps.data[1][2]
                path = self.swing_foot_path(self.init_r_sole_xyz, self.goal_r_sole_xyz, self.max_swing_height, self.t_bez)
                self.current_r_sole_xyz_yaw[0] = path[0,0]
                self.current_r_sole_xyz_yaw[1] = path[1,0]
                self.current_r_sole_xyz_yaw[2] = path[2,0]
                yaw_path = self.rot_path(self.init_r_sole_yaw, self.goal_r_sole_yaw, self.t_ssp, self.t_bez)
                self.current_r_sole_xyz_yaw[3] = yaw_path[1,0]
            elif self.support_foot == SupportFoot.RIGHT_SUPPORT:
                self.walking_phase = WalkingPhase.SSP_RIGHT
                self.current_r_sole_xyz_yaw[0] = self.footsteps.data[1][0]
                self.current_r_sole_xyz_yaw[1] = self.footsteps.data[1][1]
                self.current_r_sole_xyz_yaw[2] = 0
                self.current_r_sole_xyz_yaw[3] = self.footsteps.data[1][2]
                path = self.swing_foot_path(self.init_l_sole_xyz, self.goal_l_sole_xyz, self.max_swing_height, self.t_bez)
                self.current_l_sole_xyz_yaw[0] = path[0,0]
                self.current_l_sole_xyz_yaw[1] = path[1,0]
                self.current_l_sole_xyz_yaw[2] = path[2,0]
                yaw_path = self.rot_path(self.init_l_sole_yaw, self.goal_l_sole_yaw, self.t_ssp, self.t_bez)
                self.current_l_sole_xyz_yaw[3] = yaw_path[1,0]

            yaw_path = self.rot_path(self.init_com_yaw, self.goal_com_yaw, self.t_ssp, self.t_bez)
            self.com_yaw = float(yaw_path[1,0])
            self.t_bez += self.dt_bez

    def get_sole_pose(self, com_xyz_yaw, current_r_sole_xyz_yaw, current_l_sole_xyz_yaw):
        roll = 0.0
        pitch = self.body_tilt
        yaw = com_xyz_yaw[3]
        
        # Disini untuk mencari posisi com yang baru yang telah di offset
        dy = self.support_x * np.sin(yaw)
        dx = self.support_x * np.cos(yaw)

        com_x = com_xyz_yaw[0] + dx 
        com_y = com_xyz_yaw[1] + dy 
        com_z = com_xyz_yaw[2]

        com_pose = [com_x, com_y, com_z, roll, pitch, yaw]
        world_to_com = self.tf_matrix_from_xyz_rpy(com_pose)
        world_to_l_sole = self.create_tf_matrix(current_l_sole_xyz_yaw)
        world_to_r_sole = self.create_tf_matrix(current_r_sole_xyz_yaw)
        world_to_com_inv = np.linalg.pinv(world_to_com)
        com_to_l_sole = world_to_com_inv.dot(world_to_l_sole)
        com_to_r_sole = world_to_com_inv.dot(world_to_r_sole)
        q_l_sole = quaternion_from_matrix(com_to_l_sole[:3,:3])
        q_r_sole = quaternion_from_matrix(com_to_r_sole[:3,:3])
        l_sole_pose = Pose()
        l_sole_pose.position.x = com_to_l_sole[0,3]
        l_sole_pose.position.y = com_to_l_sole[1,3]
        l_sole_pose.position.z = com_to_l_sole[2,3]
        l_sole_pose.orientation.x = q_l_sole[1]
        l_sole_pose.orientation.y = q_l_sole[2]
        l_sole_pose.orientation.z = q_l_sole[3]
        l_sole_pose.orientation.w = q_l_sole[0]
        r_sole_pose = Pose()
        r_sole_pose.position.x = com_to_r_sole[0,3]
        r_sole_pose.position.y = com_to_r_sole[1,3]
        r_sole_pose.position.z = com_to_r_sole[2,3]
        r_sole_pose.orientation.x = q_r_sole[1]
        r_sole_pose.orientation.y = q_r_sole[2]
        r_sole_pose.orientation.z = q_r_sole[3]
        r_sole_pose.orientation.w = q_r_sole[0]
        return r_sole_pose, l_sole_pose

    def publish_current_state(self):
        if self.state == GaitControllerState.INITIALIZE:
            state = "init"
        elif self.state == GaitControllerState.START:
            state = "start"
        elif self.state == GaitControllerState.STOP:
            state = "stop"
        current_state = WalkingState()
        current_state.state = state
        self.gait_state_pub.publish(current_state)

    def run(self):
        rospy.loginfo("========================")
        rospy.loginfo("%s", self)
        rospy.loginfo("========================")
        if self.simulation_mode:
            rospy.loginfo("Running Mode : Gazebo")
        else:
            rospy.loginfo("Running Mode : Robot")
        rospy.loginfo("t_step : %s", self.t_step)
        rospy.loginfo("dt : %s", self.dt)
        rospy.loginfo("DSP Ratio : %s", self.dsp_ratio)
        rospy.loginfo("Max Swing Height : %s", self.max_swing_height)
        rospy.loginfo("Support x : %s", self.support_x)
        rospy.loginfo("Support y : %s", self.support_y)
        rospy.loginfo("Body Tilt : %s", self.body_tilt)
        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.spin()

    def print_gait_param(self):
        rospy.loginfo("<< Gait Param >>")
        rospy.loginfo("t_step >> %s", self.t_step)
        rospy.loginfo("dsp_ratio >> %s", self.dsp_ratio)
        rospy.loginfo("swing_foot_height >> %s", self.max_swing_height)
        rospy.loginfo("support_x >> %s", self.support_x)
        rospy.loginfo("support_y >> %s", self.support_y)
        rospy.loginfo("body_tilt >> %s", self.body_tilt)

    # Timer Callback
    def timer_callback(self, event):
        if self.state == GaitControllerState.INITIALIZE:
            if self.robot_standing:
                if self.command == "start":
                    self.print_gait_param()
                    self.state = GaitControllerState.START
            else:
                self.reset_gait()
                motion_data = PlayMotionGoal()
                motion_data.motion_name = "initialize"
                motion_data.movement_time = 2.0
                self.play_motion_ac.send_goal(motion_data)
                self.play_motion_ac.wait_for_result()

                success = self.play_motion_ac.get_result()

                motion_data = PlayMotionGoal()
                motion_data.motion_name = "standing"
                motion_data.movement_time = 2.0
                self.play_motion_ac.send_goal(motion_data)
                self.play_motion_ac.wait_for_result()

                success = self.play_motion_ac.get_result()

                if success:
                    self.robot_standing = True
                else:
                    self.robot_standing = False
                    rospy.logwarn("Failed call motion player >> '%s'", motion_data.motion_name)

        elif self.state == GaitControllerState.START:
            self.get_walking_pattern()
            if self.command == "stop":
                # walking di tempat dulu
                if self.cmd_x != 0 or self.cmd_y != 0 or self.cmd_alpha != 0:
                    self.cmd_x = 0
                    self.cmd_y = 0
                    self.cmd_alpha = 0
                
                if self.stop_signal:
                    success = True
                    if success:
                        self.state = GaitControllerState.STOP
                    else:
                        rospy.logwarn("Failed call motion player >> '%s'", motion.motion_name)
                    
        elif self.state == GaitControllerState.STOP:
            if self.command == "start":
                self.print_gait_param()
                self.state = GaitControllerState.START
            elif self.command == "init":
                self.robot_standing = False
                self.state = GaitControllerState.INITIALIZE
        
        self.publish_current_state()

class ZMPAnalyticalController(GaitController):
    def __init__(self, node_name):
        super(ZMPAnalyticalController, self).__init__(node_name)

    def load_timing_param(self):
        rospy.loginfo("Load timing param")
        self.zc = rospy.get_param("/gait_param/lipm_height")
        self.zc = self.rb_kinematics.PELVIS_TO_SOLE - 0.015
        if self.simulation_mode:
            self.t_step = rospy.get_param("/gait_param/gazebo/t_step")
            self.dt = rospy.get_param("/gait_param/gazebo/control_cycle")
            self.dsp_ratio = rospy.get_param("/gait_param/gazebo/dsp_ratio")
        else:
            self.t_step = rospy.get_param("/gait_param/robot/t_step")
            self.dt = rospy.get_param("/gait_param/robot/control_cycle")
            self.dsp_ratio = rospy.get_param("/gait_param/robot/dsp_ratio")
       
        self.t_dsp = self.dsp_ratio * self.t_step
        self.t_ssp = (1.0 - self.dsp_ratio) * self.t_step
        self.dt_bez = 1 / (self.t_ssp / self.dt)

    def reset_gait(self):
        rospy.loginfo("<< RESET GAIT >>")
        self.load_param()
        self.load_timing_param()
        self.init_gait()

    def get_zmp_trajectory(self):
        epsilon = 0.0001 
        td = self.t % self.t_step 
        if td > -epsilon and td < epsilon:
            self.t0 = self.t
            self.t1 = self.t0 + (self.t_ssp / 2)
            self.t2 = self.t1 + self.t_dsp
            self.tf = self.t_step
            # Initial CoM position
            self.com0_x = self.footsteps.data[0][0] + (self.footsteps.data[1][0] - self.footsteps.data[0][0]) / 2
            self.com0_y = self.footsteps.data[0][1] + (self.footsteps.data[1][1] - self.footsteps.data[0][1]) / 2
            # Final CoM position
            self.com1_x = self.footsteps.data[1][0] + (self.footsteps.data[2][0] - self.footsteps.data[1][0]) / 2
            self.com1_y = self.footsteps.data[1][1] + (self.footsteps.data[2][1] - self.footsteps.data[1][1]) / 2
            # Support foot
            self.sup_x = self.footsteps.data[1][0]
            self.sup_y = self.footsteps.data[1][1]

        if self.t >= self.t0 and self.t < self.t1:
            self.zmp_x = self.com0_x+((self.sup_x-self.com0_x)/(self.t1-self.t0))*self.t
            self.zmp_y = self.com0_y+((self.sup_y-self.com0_y)/(self.t1-self.t0))*self.t
        elif self.t >= self.t1 and self.t < self.t2:
            self.zmp_x = self.sup_x
            self.zmp_y = self.sup_y 
        elif self.t >= self.t2 and self.t < self.tf:
            self.zmp_x=self.sup_x+((self.com1_x-self.sup_x)/(self.tf-self.t2))*(self.t-self.t2)
            self.zmp_y=self.sup_y+((self.com1_y-self.sup_y)/(self.tf-self.t2))*(self.t-self.t2)
    
    def get_com_trajectory(self):
        self.Tc = np.sqrt(9.81/self.zc)
        cx = np.array([0,
                       (np.sinh(self.Tc*(self.t1 - self.tf))*(self.sup_x - self.com0_x))/(self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - (np.sinh(self.Tc*(self.t2 - self.tf))*(self.sup_x - self.com1_x))/(self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.cosh(self.Tc*(2*self.t1 - self.tf)) - np.cosh(self.Tc*self.tf))*(self.sup_x - self.com0_x))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.cosh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.cosh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_x - self.com1_x))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.sinh(self.Tc*(2*self.t1 - self.tf)) + np.sinh(self.Tc*self.tf))*(self.sup_x - self.com0_x))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.sinh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.sinh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_x - self.com1_x))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.cosh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.cosh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_x - self.com0_x))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.cosh(self.Tc*(2*self.t2 - self.tf)) - np.cosh(self.Tc*self.tf))*(self.sup_x - self.com1_x))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((self.sup_x - self.com0_x)*(np.sinh(self.Tc*(self.t1 + self.t2 - self.tf)) + np.sinh(self.Tc*(self.t1 - self.t2 + self.tf))))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.sinh(self.Tc*(2*self.t2 - self.tf)) + np.sinh(self.Tc*self.tf))*(self.sup_x - self.com1_x))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf))])

        cy = np.array([0,
                       (np.sinh(self.Tc*(self.t1 - self.tf))*(self.sup_y - self.com0_y))/(self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - (np.sinh(self.Tc*(self.t2 - self.tf))*(self.sup_y - self.com1_y))/(self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.cosh(self.Tc*(2*self.t1 - self.tf)) - np.cosh(self.Tc*self.tf))*(self.sup_y - self.com0_y))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.cosh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.cosh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_y - self.com1_y))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.sinh(self.Tc*(2*self.t1 - self.tf)) + np.sinh(self.Tc*self.tf))*(self.sup_y - self.com0_y))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.sinh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.sinh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_y - self.com1_y))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((np.cosh(self.Tc*(self.t1 + self.t2 - self.tf)) - np.cosh(self.Tc*(self.t1 - self.t2 + self.tf)))*(self.sup_y - self.com0_y))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.cosh(self.Tc*(2*self.t2 - self.tf)) - np.cosh(self.Tc*self.tf))*(self.sup_y - self.com1_y))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf)),
                       ((self.sup_y - self.com0_y)*(np.sinh(self.Tc*(self.t1 + self.t2 - self.tf)) + np.sinh(self.Tc*(self.t1 - self.t2 + self.tf))))/(2*self.Tc*self.t1*np.sinh(self.Tc*self.tf)) - ((np.sinh(self.Tc*(2*self.t2 - self.tf)) + np.sinh(self.Tc*self.tf))*(self.sup_y - self.com1_y))/(2*self.Tc*np.sinh(self.Tc*self.tf)*(self.t2 - self.tf))])

        if self.t >= self.t0 and self.t < self.t1:
            self.com_xyz_yaw[0] = self.com0_x+((self.sup_x-self.com0_x)/(self.t1-self.t0))*(self.t-self.t0)+cx[0]*np.cosh(self.Tc*self.t)+cx[1]*np.sinh(self.Tc*self.t)
            self.com_xyz_yaw[1] = self.com0_y+((self.sup_y-self.com0_y)/(self.t1-self.t0))*(self.t-self.t0)+cy[0]*np.cosh(self.Tc*self.t)+cy[1]*np.sinh(self.Tc*self.t)
        elif self.t >= self.t1 and self.t < self.t2:
            self.com_xyz_yaw[0] = self.sup_x+cx[2]*np.cosh(self.Tc*(self.t-self.t1))+cx[3]*np.sinh(self.Tc*(self.t-self.t1))
            self.com_xyz_yaw[1] = self.sup_y+cy[2]*np.cosh(self.Tc*(self.t-self.t1))+cy[3]*np.sinh(self.Tc*(self.t-self.t1))
        elif self.t >= self.t2 and self.t < self.tf:
            self.com_xyz_yaw[0] = self.sup_x+((self.com1_x-self.sup_x)/(self.tf-self.t2))*(self.t-self.t2)+cx[4]*np.cosh(self.Tc*(self.t-self.t2))+cx[5]*np.sinh(self.Tc*(self.t-self.t2))
            self.com_xyz_yaw[1] = self.sup_y+((self.com1_y-self.sup_y)/(self.tf-self.t2))*(self.t-self.t2)+cy[4]*np.cosh(self.Tc*(self.t-self.t2))+cy[5]*np.sinh(self.Tc*(self.t-self.t2))

        self.com_xyz_yaw[2] = self.zc
        self.com_xyz_yaw[3] = self.com_yaw

    def get_walking_pattern(self):
        if self.footsteps.is_full():
            self.get_zmp_trajectory()
            self.get_com_trajectory()
            self.get_sole_trajectory()
            com = self.com_xyz_yaw
            r_sole = self.current_r_sole_xyz_yaw
            l_sole = self.current_l_sole_xyz_yaw
            r_sole_pose, l_sole_pose = self.get_sole_pose(com, r_sole, l_sole)
            rospy.loginfo("left sole %s", l_sole_pose)
            rospy.loginfo("right sole %s", r_sole_pose)
            # self.set_leg_pose(r_sole_pose, l_sole_pose)
            # disini calc IK sama send to joint
            success = True
            try:
                l_leg_response = self.calc_l_leg_ik_srv(l_sole_pose)
                r_leg_response = self.calc_r_leg_ik_srv(r_sole_pose)
            except rospy.ServiceException as e:
                rospy.logwarn("Calc IK Service Failed :" + str(e))
                # Call Move Body
                success = False
            if l_leg_response.success and r_leg_response.success:
                q_leg_goal = np.hstack((l_leg_response.q_out, r_leg_response.q_out))
                self.rb_joint_states_pub.set_leg(q_leg_goal)
            # Check stop command
            epsilon = 0.0001
            # delta_z = abs(l_sole_pose[2] - r_sole_pose[2])
            # delta_x = abs(l_sole_pose[0] - r_sole_pose[0])
            delta_z = abs(l_sole_pose.position.z - r_sole_pose.position.z)
            delta_x = abs(l_sole_pose.position.x - r_sole_pose.position.x)
            if delta_x < epsilon and delta_z < epsilon:
                self.stop_signal = True 
            else:
                self.stop_signal = False

        self.t += self.dt 
        if self.t > self.t_step:
            self.t = 0
            self.add_new_footstep()

    def __str__(self):
        _str = "ZMP Analytical Controller"
        return _str

def main():
    node_name = "gait_controller_node"
    gc = ZMPAnalyticalController(node_name)
    gc.run()

if __name__ == "__main__":
    main()