#!/usr/bin/env python

import rospy
import abc 
import numpy as np 
import math as m
import actionlib
import PyKDL as kdl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from robinion_msgs.srv import CalcFK, CalcIK
from robinion_msgs.msg import PlayMotionAction, PlayMotionFeedback, PlayMotionResult
from tf_conversions import posemath as pm
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kinematics_pkg.kinematics import Kinematics
from trajectory_generator_pkg.trajectory_generator import TrajectoryGenerator
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher

class MotionPlayer():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        # Initialization
        self.rb_joint_states_data = JointStatesSubscriber()
        self.rb_joint_states_pub = JointStatesPublisher()
        self.rb_kinematics = Kinematics()
        self.trajectory_generator = TrajectoryGenerator()
        # Comm Initialization
        self.init_subscriber()
        self.init_publisher()
        self.init_service_server()
        self.init_service_client()
        self.init_action_server()
        self.init_action_client()

    def init_subscriber(self):
        pass

    def init_publisher(self):
        pass

    def init_service_server(self):
        pass

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_ik")
        self.calc_l_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_fk", CalcFK)
        self.calc_r_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_fk", CalcFK)
        self.calc_l_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_fk", CalcFK)
        self.calc_r_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_fk", CalcFK)
        self.calc_l_arm_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_ik", CalcIK)
        self.calc_r_arm_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_ik", CalcIK)
        self.calc_l_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_ik", CalcIK)
        self.calc_r_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_ik", CalcIK)

    def init_action_server(self):
        self.play_motion_as = actionlib.SimpleActionServer("/robinion/motion_player/play_motion", 
                                                            PlayMotionAction, execute_cb=self.play_motion_action_callback, 
                                                            auto_start=False)
        self.play_motion_as.start()

    def init_action_client(self):
        pass

    def play_motion_action_callback(self, motion):
        success = False
        feedback = PlayMotionFeedback()
        result = PlayMotionResult()
        rospy.loginfo("Play Motion request >> '%s' with movement time %s seconds", motion.motion_name, motion.movement_time)
        # Load current param
        feedback.is_running = True
        self.play_motion_as.publish_feedback(feedback)
        if motion.motion_name == "zero":
            if motion.movement_time == 0:
                success = self.zero()
            else:
                success = self.zero(movement_time = motion.movement_time)
        elif motion.motion_name == "initialize":
            if motion.movement_time == 0:
                success = self.initialize()
            else:
                success = self.initialize(movement_time = motion.movement_time)
        elif motion.motion_name == "standing":
            if motion.movement_time == 0:
                success = self.standing()
            else:
                success = self.standing(movement_time = motion.movement_time)
        elif motion.motion_name == "sitting":
            if motion.movement_time == 0:
                success = self.sitting()
            else:
                success = self.sitting(movement_time = motion.movement_time)
        else:
            success = False
        feedback.is_running = False
        self.play_motion_as.publish_feedback(feedback)
        if success:
            rospy.loginfo("Play Motion Request >> '%s' with movement time '%s' seconds >> Success", motion.motion_name, motion.movement_time)
        else:
            rospy.loginfo("Play Motion Request >> '%s' with movement time '%s' seconds >> Failed", motion.motion_name, motion.movement_time)
        result.success = success
        self.play_motion_as.set_succeeded(result)

    def set_pelvis_pose(self, pelvis_pose, movement_time=1.0, dt=0.01):
        pelvis_q = [pelvis_pose.orientation.x, pelvis_pose.orientation.y, pelvis_pose.orientation.z, pelvis_pose.orientation.w]
        (_, pelvis_pitch, _) = euler_from_quaternion (pelvis_q)
        # Convert pose to frame
        pelvis_frame = pm.fromMsg(pelvis_pose)
        # Set left sole frame
        l_sole_frame = kdl.Frame(kdl.Rotation.RPY(0,0,0), kdl.Vector(0, (self.rb_kinematics.HIP_TO_CROTCH+0.05), 0))
        # Set right sole frame
        r_sole_frame = kdl.Frame(kdl.Rotation.RPY(0,0,0), kdl.Vector(0, -(self.rb_kinematics.HIP_TO_CROTCH+0.05), 0))

        pelvis_frame_inv = pelvis_frame.Inverse()
        l_sole_from_pelvis = pelvis_frame_inv * l_sole_frame
        r_sole_from_pelvis = pelvis_frame_inv * r_sole_frame

        l_sole_pose = pm.toMsg(l_sole_from_pelvis)
        r_sole_pose = pm.toMsg(r_sole_from_pelvis)
        rospy.loginfo("l sole %s", l_sole_pose)
        rospy.loginfo("r sole %s", r_sole_pose)

        # Calc IK Leg
        success = True
        try:
            l_leg_response = self.calc_l_leg_ik_srv(l_sole_pose)
            r_leg_response = self.calc_r_leg_ik_srv(r_sole_pose)
        except rospy.ServiceException as e:
            rospy.logwarn("Calc IK Service Failed :" + str(e))
            # Call Move Body
            success = False
        if l_leg_response.success and r_leg_response.success:
            q_leg_init = self.rb_joint_states_data.q_leg.copy()
            q_leg_goal = np.hstack((l_leg_response.q_out, r_leg_response.q_out))
            # Disini execute motion
            q_leg_traj = self.trajectory_generator.minimum_jerk(q_leg_init, q_leg_goal, d=movement_time, dt=dt)
            t = 0.0
            for i in range(q_leg_traj.shape[0]):
                # Here we publish to joint
                self.rb_joint_states_pub.set_leg(q_leg_traj[i])
                t += dt
                # Take sleep
                rospy.sleep(dt)
        return success

    def zero(self, movement_time=1.0, dt=0.01):
        q_full_body_init = self.rb_joint_states_data.q_full_body.copy()
        q_full_body_goal = np.zeros(q_full_body_init.shape)
        q_full_body_traj = self.trajectory_generator.minimum_jerk(q_full_body_init, q_full_body_goal, d=movement_time, dt=dt)
        t = 0.0
        for i in range(q_full_body_traj.shape[0]):
            # Here we publish to joint
            self.rb_joint_states_pub.set_full_body(q_full_body_traj[i])
            t += dt
            # Take sleep
            rospy.sleep(dt)
        return True
    
    def initialize(self, movement_time=1.0, dt=0.01):
        q_full_body_init = self.rb_joint_states_data.q_full_body.copy()
        q_full_body_goal = np.zeros(q_full_body_init.shape)
        # Set shoulder to down
        q_full_body_goal[12] = -np.pi/2
        q_full_body_goal[16] = np.pi/2
        q_full_body_traj = self.trajectory_generator.minimum_jerk(q_full_body_init, q_full_body_goal, d=movement_time, dt=dt)
        t = 0.0
        for i in range(q_full_body_traj.shape[0]):
            # Here we publish to joint
            self.rb_joint_states_pub.set_full_body(q_full_body_traj[i])
            t += dt
            # Take sleep
            rospy.sleep(dt)
        return True
    
    def standing(self, movement_time=1.0):
        pelvis_pose = Pose()
        pelvis_pose.position.x = 0
        pelvis_pose.position.y = 0
        pelvis_pose.position.z = self.rb_kinematics.PELVIS_TO_SOLE - 0.015
        pelvis_pose.orientation.x = 0
        pelvis_pose.orientation.y = 0
        pelvis_pose.orientation.z = 0
        pelvis_pose.orientation.w = 1
        return self.set_pelvis_pose(pelvis_pose, movement_time=movement_time)
    
    # Sitting belum fix
    def sitting(self, movement_time=1.0):
        pelvis_pose = Pose()
        pelvis_pose.position.x = 0
        pelvis_pose.position.y = 0
        pelvis_pose.position.z = 0.250
        q = quaternion_from_euler(0, np.radians(0), 0)
        pelvis_pose.orientation.x = q[0]
        pelvis_pose.orientation.y = q[1]
        pelvis_pose.orientation.z = q[2]
        pelvis_pose.orientation.w = q[3]
        return self.set_pelvis_pose(pelvis_pose, movement_time=movement_time)
    
    def run(self):
        rospy.loginfo("========================")
        rospy.loginfo(" Robinion Motion Player ")
        rospy.loginfo("========================")
        rospy.spin()

def main():
    node_name = "motion_player_node"
    motion_player = MotionPlayer(node_name)
    motion_player.run()

if __name__ == "__main__":
    main()