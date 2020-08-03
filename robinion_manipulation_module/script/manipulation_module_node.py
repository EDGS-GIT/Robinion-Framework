#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from robinion_msgs.srv import CalcFK, CalcIK
from robinion_msgs.msg import MoveArmAction, MoveArmFeedback, MoveArmResult
from trajectory_generator_pkg.trajectory_generator import TrajectoryGenerator
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher

class ManipulationModule():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        # Initialization
        self.rb_joint_states_data = JointStatesSubscriber()
        self.rb_joint_states_pub = JointStatesPublisher()
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
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_ik")
        self.calc_l_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_fk", CalcFK)
        self.calc_r_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_fk", CalcFK)
        self.calc_l_arm_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_ik", CalcIK)
        self.calc_r_arm_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_ik", CalcIK)

    def init_action_server(self):
        self.move_l_arm_as = actionlib.SimpleActionServer("/robinion/manipulation_module/move_l_arm", 
                                                          MoveArmAction, execute_cb=self.move_l_arm_action_callback,
                                                          auto_start=False)
        self.move_r_arm_as = actionlib.SimpleActionServer("/robinion/manipulation_module/move_r_arm", 
                                                          MoveArmAction, execute_cb=self.move_r_arm_action_callback,
                                                          auto_start=False)
        self.move_l_arm_as.start()
        self.move_r_arm_as.start()

    def init_action_client(self):
        pass
    
    def move_l_arm_action_callback(self, data):
        feedback = MoveArmFeedback()
        result = MoveArmResult()
        # Calculate IK
        response = self.calc_l_arm_ik_srv(data.target_pose)
        if response.success:
            rospy.loginfo("Execute motion")
            q_l_arm_init = self.rb_joint_states_data.q_l_arm_with_torso.copy()
            q_l_arm_goal = response.q_out
            success = True
            
            l_arm_traj = self.trajectory_generator.minimum_jerk(q_l_arm_init, q_l_arm_goal, d=data.movement_time, dt=data.dt)
            t = 0.0

            for i in range(l_arm_traj.shape[0]):
                if self.move_l_arm_as.is_preempt_requested():
                    success = False
                    break
                # Here we publish the feedback of l arm pose
                response = self.calc_l_arm_fk_srv(l_arm_traj[i])
                feedback.current_pose = response.pose
                feedback.current_time = t
                self.move_l_arm_as.publish_feedback(feedback)
                # Here we publish to joint 
                self.rb_joint_states_pub.set_l_arm_with_torso(l_arm_traj[i])
                t += data.dt
                # Take sleep
                rospy.sleep(data.dt)

            result.success = success
            self.move_l_arm_as.set_succeeded(result)
        else:
            rospy.loginfo("IK Failed")
            result.success = False
            self.move_l_arm_as.set_succeeded(result)
    
    def move_r_arm_action_callback(self, data):
        feedback = MoveArmFeedback()
        result = MoveArmResult()
        # Calculate IK
        response = self.calc_r_arm_ik_srv(data.target_pose)
        if response.success:
            rospy.loginfo("Execute motion")
            q_r_arm_init = self.rb_joint_states_data.q_r_arm_with_torso.copy()
            q_r_arm_goal = response.q_out
            success = True
            
            r_arm_traj = self.trajectory_generator.minimum_jerk(q_r_arm_init, q_r_arm_goal, d=data.movement_time, dt=data.dt)
            t = 0.0

            for i in range(r_arm_traj.shape[0]):
                if self.move_r_arm_as.is_preempt_requested():
                    success = False
                    break
                # Here we publish the feedback of l arm pose
                response = self.calc_r_arm_fk_srv(r_arm_traj[i])
                feedback.current_pose = response.pose
                feedback.current_time = t
                self.move_r_arm_as.publish_feedback(feedback)
                # Here we publish to joint 
                self.rb_joint_states_pub.set_r_arm_with_torso(r_arm_traj[i])
                t += data.dt
                # Take sleep
                rospy.sleep(data.dt)

            result.success = success
            self.move_r_arm_as.set_succeeded(result)
        else:
            rospy.loginfo("IK Failed")
            result.success = False
            self.move_r_arm_as.set_succeeded(result)
    
    def run(self):
        rospy.loginfo("==============================")
        rospy.loginfo(" Robinion Manipulation Module ")
        rospy.loginfo("==============================")
        rospy.spin()

def main():
    node_name = "manipulation_module_node"
    manipulation_module = ManipulationModule(node_name)
    manipulation_module.run()

if __name__ == "__main__":
    main()