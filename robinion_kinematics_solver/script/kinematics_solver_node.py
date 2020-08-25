#!/usr/bin/env python

import rospy
import numpy as np
# import Py3KDL as kdl
import PyKDL as kdl
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Imu, JointState
from robinion_msgs.srv import CalcFK, CalcIK
import kdl_parser_py.urdf as kdl_parser
import math as m
from utility_pkg.utility import Utility
from leg_kinematics import LegKinematics
from utility_pkg.utility import Utility
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber

class KinematicsSolver():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        self.utility = Utility()
        self.rb_joint_states_data = JointStatesSubscriber()
        # Init Comm
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
        # Forward Kinematics Service
        rospy.Service("/robinion/kinematics_solver/calc_head_fk", CalcFK, self.handle_calc_head_fk_service)
        rospy.Service("/robinion/kinematics_solver/calc_l_arm_fk", CalcFK, self.handle_calc_l_arm_fk_service)
        rospy.Service("/robinion/kinematics_solver/calc_r_arm_fk", CalcFK, self.handle_calc_r_arm_fk_service)
        rospy.Service("/robinion/kinematics_solver/calc_l_leg_fk", CalcFK, self.handle_calc_l_leg_fk_service)
        rospy.Service("/robinion/kinematics_solver/calc_r_leg_fk", CalcFK, self.handle_calc_r_leg_fk_service)
        # Inverse Kinematics Service
        rospy.Service("/robinion/kinematics_solver/calc_l_arm_ik", CalcIK, self.handle_calc_l_arm_ik_service)
        rospy.Service("/robinion/kinematics_solver/calc_r_arm_ik", CalcIK, self.handle_calc_r_arm_ik_service)
        rospy.Service("/robinion/kinematics_solver/calc_l_leg_ik", CalcIK, self.handle_calc_l_leg_ik_service)
        rospy.Service("/robinion/kinematics_solver/calc_r_leg_ik", CalcIK, self.handle_calc_r_leg_ik_service)

    def init_service_client(self):
        pass

    def init_action_server(self):
        pass

    def init_action_client(self):
        pass

class ForwardKinematics(KinematicsSolver):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.urdf_string = rospy.get_param("/robot_description")
        self.init_kdl_chain()
        self.init_forward_kinematics_upper_body()
        self.utility = Utility()
        self.leg_kinematics_solver = LegKinematics()

    def init_kdl_chain(self):
        (ok, tree) = kdl_parser.treeFromString(self.urdf_string)
        if not ok:
            rospy.logerr("Failed parsing URDF")

        # Head Chain
        self.head_chain = tree.getChain("torso_link", "gaze_link")
        head_chain_n_joints = self.head_chain.getNrOfJoints()
        rospy.loginfo("Head n-Joint : %d", head_chain_n_joints)
        for i in range(head_chain_n_joints):
            rospy.loginfo("Segment : %s", self.head_chain.getSegment(i).getName())
            rospy.loginfo("Joint : %s", self.head_chain.getSegment(i).getJoint().getName())
        rospy.loginfo("============================")

        # Left Arm Chain
        self.l_arm_chain = tree.getChain("torso_link", "l_end_effector")
        l_arm_chain_n_joints = self.l_arm_chain.getNrOfJoints()
        rospy.loginfo("Left Arm n-Joint : %d", l_arm_chain_n_joints)
        for i in range(l_arm_chain_n_joints):
            rospy.loginfo("Segment : %s", self.l_arm_chain.getSegment(i).getName())
            rospy.loginfo("Joint : %s", self.l_arm_chain.getSegment(i).getJoint().getName())
        rospy.loginfo("============================")

        # Right Arm Chain
        self.r_arm_chain = tree.getChain("torso_link", "r_end_effector")
        r_arm_chain_n_joints = self.r_arm_chain.getNrOfJoints()
        rospy.loginfo("Right Arm n-Joint : %d", r_arm_chain_n_joints)
        for i in range(r_arm_chain_n_joints):
            rospy.loginfo("Segment : %s", self.r_arm_chain.getSegment(i).getName())
            rospy.loginfo("Joint : %s", self.r_arm_chain.getSegment(i).getJoint().getName())
        rospy.loginfo("============================")

    def init_forward_kinematics_upper_body(self):
        self.head_fk = kdl.ChainFkSolverPos_recursive(self.head_chain)
        self.l_arm_fk = kdl.ChainFkSolverPos_recursive(self.l_arm_chain)
        self.r_arm_fk = kdl.ChainFkSolverPos_recursive(self.r_arm_chain)

    def fk_head(self, q_head_kdl):
        eef_frame_kdl = kdl.Frame()
        self.head_fk.JntToCart(q_head_kdl, eef_frame_kdl)
        return eef_frame_kdl
    
    def fk_l_arm(self, q_l_arm_kdl):
        eef_frame_kdl = kdl.Frame()
        self.l_arm_fk.JntToCart(q_l_arm_kdl, eef_frame_kdl)
        return eef_frame_kdl

    def fk_r_arm(self, q_r_arm_kdl):
        eef_frame_kdl = kdl.Frame()
        self.r_arm_fk.JntToCart(q_r_arm_kdl, eef_frame_kdl)
        return eef_frame_kdl
    
    # def fk_l_leg(self, q_l_leg_kdl):
    #     pass

    # def fk_r_leg(self, q_r_leg_kdl):
    #     pass
    
    def handle_calc_head_fk_service(self, req):
        rospy.loginfo("Request >> Get head pose")
        q = self.utility.ros_array_to_kdl_jnt_array(req.q_in)
        eef_frame_kdl = self.fk_head(q)
        eef_pose = self.utility.kdl_frame_to_ros_pose(eef_frame_kdl)
        return eef_pose

    def handle_calc_l_arm_fk_service(self, req):
        rospy.loginfo("Request >> Get left arm pose")
        q = self.utility.ros_array_to_kdl_jnt_array(req.q_in)
        eef_frame_kdl = self.fk_l_arm(q)
        eef_pose = self.utility.kdl_frame_to_ros_pose(eef_frame_kdl)
        return eef_pose

    def handle_calc_r_arm_fk_service(self, req):
        rospy.loginfo("Request >> Get right arm pose")
        q = self.utility.ros_array_to_kdl_jnt_array(req.q_in)
        eef_frame_kdl = self.fk_r_arm(q)
        eef_pose = self.utility.kdl_frame_to_ros_pose(eef_frame_kdl)
        return eef_pose
    
    def handle_calc_l_leg_fk_service(self, req):
        rospy.loginfo("Request >> Get left leg pose")
        q = self.leg_kinematics_solver.get_full_leg_joints(req.q_in)
        eef_frame_kdl = self.leg_kinematics_solver.fk_l_leg(q)
        eef_pose = self.utility.kdl_frame_to_ros_pose(eef_frame_kdl)
        return eef_pose
    
    def handle_calc_r_leg_fk_service(self, req):
        rospy.loginfo("Request >> Get right leg pose")
        q = self.leg_kinematics_solver.get_full_leg_joints(req.q_in)
        eef_frame_kdl = self.leg_kinematics_solver.fk_r_leg(q)
        eef_pose = self.utility.kdl_frame_to_ros_pose(eef_frame_kdl)
        return eef_pose

class InverseKinematics(ForwardKinematics):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.init_kdl_ik()

    def init_kdl_ik(self):
        min_error = rospy.get_param("/kinematics_solver/min_error")
        max_iter = rospy.get_param("/kinematics_solver/max_iter")

        # Bounds
        self.lower = kdl.JntArray(5)
        self.lower[0] = -np.pi*0.5
        self.lower[1] = -np.pi*0.9
        self.lower[2] = -np.pi*0.5
        self.lower[3] = -np.pi*0.9
        self.lower[4] = -np.pi*0.5

        self.upper = kdl.JntArray(5)
        self.upper[0] = np.pi*0.5
        self.upper[1] = np.pi*0.9
        self.upper[2] = np.pi*0.5
        self.upper[3] = np.pi*0.9
        self.upper[4] = np.pi*0.5

        # Left Arms
        self.l_arm_vik = kdl.ChainIkSolverVel_pinv(self.l_arm_chain)
        self.l_arm_ik = kdl.ChainIkSolverPos_NR_JL(self.l_arm_chain, self.lower, self.upper, self.l_arm_fk, self.l_arm_vik, maxiter=max_iter, eps=min_error)

        # Right Arm
        self.r_arm_vik = kdl.ChainIkSolverVel_pinv(self.r_arm_chain)
        self.r_arm_ik = kdl.ChainIkSolverPos_NR_JL(self.r_arm_chain, self.lower, self.upper, self.r_arm_fk, self.r_arm_vik, maxiter=max_iter, eps=min_error)

    # Inverse Kinematics Method
    def ik_l_arm(self, target_frame):
        q_out = kdl.JntArray(5)
        q_init = self.utility.ros_array_to_kdl_jnt_array(self.rb_joint_states_data.q_l_arm_with_torso.copy())
        result = self.l_arm_ik.CartToJnt(q_init, target_frame, q_out)
        if result < 0:
            rospy.logerr("Left Arm IK Solver failed ")
        return result, q_out

    def ik_r_arm(self, target_frame):
        q_out = kdl.JntArray(5)
        q_init = self.utility.ros_array_to_kdl_jnt_array(self.rb_joint_states_data.q_r_arm_with_torso.copy())
        result = self.r_arm_ik.CartToJnt(q_init, target_frame, q_out)
        if result < 0:
            rospy.logerr("Right Arm IK Solver failed ")
        return result, q_out

    def handle_calc_l_arm_ik_service(self, req):
        rospy.loginfo("Request >> Calc left arm IK")
        target_frame = self.utility.ros_pose_to_kdl_frame(req.pose)
        result, q_out = self.ik_l_arm(target_frame)
        result_frame = self.fk_l_arm(q_out)
        error = self.utility.calc_error(target_frame, result_frame)
        rospy.loginfo("Error : %s", error)
        q_result = self.utility.kdl_jnt_array_to_ros_array(q_out)
        if result < 0:
            return False, q_result
        else:
            return True, q_result

    def handle_calc_r_arm_ik_service(self, req):
        rospy.loginfo("Request >> Calc right arm IK")
        target_frame = self.utility.ros_pose_to_kdl_frame(req.pose)
        result, q_out = self.ik_r_arm(target_frame)
        result_frame = self.fk_r_arm(q_out)
        error = self.utility.calc_error(target_frame, result_frame)
        rospy.loginfo("Error : %s", error)
        q_result = self.utility.kdl_jnt_array_to_ros_array(q_out)
        if result < 0:
            return False, q_result
        else:
            return True, q_result

    def handle_calc_l_leg_ik_service(self, req):
        rospy.loginfo("Request >> Calc left leg IK")
        target_frame = self.utility.ros_pose_to_kdl_frame(req.pose)
        q_result = self.leg_kinematics_solver.calc_l_leg_ik(req.pose)
        q_kdl_jnt_array = self.leg_kinematics_solver.get_full_leg_joints(q_result)
        result_frame = self.leg_kinematics_solver.fk_l_leg(q_kdl_jnt_array)
        error = self.utility.calc_error(target_frame, result_frame)
        rospy.loginfo("Error : %s", error)
        # q_result = self.utility.kdl_jnt_array_to_ros_array(q_out)
        result = 0
        if result < 0:
            return False, q_result
        else:
            return True, q_result

    def handle_calc_r_leg_ik_service(self, req):
        rospy.loginfo("Request >> Calc right leg IK")
        target_frame = self.utility.ros_pose_to_kdl_frame(req.pose)
        q_result = self.leg_kinematics_solver.calc_r_leg_ik(req.pose)
        q_kdl_jnt_array = self.leg_kinematics_solver.get_full_leg_joints(q_result)
        result_frame = self.leg_kinematics_solver.fk_r_leg(q_kdl_jnt_array)
        error = self.utility.calc_error(target_frame, result_frame)
        rospy.loginfo("Error : %s", error)
        # q_result = self.utility.kdl_jnt_array_to_ros_array(q_out)
        result = 0
        if result < 0:
            return False, q_result
        else:
            return True, q_result

    def run(self):
        rospy.loginfo("============================")
        rospy.loginfo(" Robinion Kinematics Solver ")
        rospy.loginfo("============================")
        rospy.spin()

def main():
    node_name = "kinematics_solver_node"
    ik_solver = InverseKinematics(node_name)
    ik_solver.run()

if __name__ == "__main__":
    main()