#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
import Py3KDL as kdl
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from robinion_msgs.srv import CalcFK, CalcIK
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
from trajectory_generator_pkg.trajectory_generator import TrajectoryGenerator
from kinematics_pkg.kinematics import Kinematics

class LegKinematicsTester():
    def __init__(self, node_name):
        super().__init__()
        self.trajectory_generator = TrajectoryGenerator()
        self.rb_kinematics = Kinematics()
        self.rb_joint_states_pub = JointStatesPublisher()
        self.rb_joint_states_data = JointStatesSubscriber()
        rospy.init_node(node_name)
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
        self.l_leg_marker_pub = rospy.Publisher("/robinion/module_tester/l_leg_marker", Marker, queue_size=1)
        self.r_leg_marker_pub = rospy.Publisher("/robinion/module_tester/r_leg_marker", Marker, queue_size=1)

    def init_service_server(self):
        pass

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_leg_ik")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_leg_ik")
        self.calc_l_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_fk", CalcFK)
        self.calc_r_leg_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_fk", CalcFK)
        self.calc_l_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_leg_ik", CalcIK)
        self.calc_r_leg_ik_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_leg_ik", CalcIK)

    def init_action_server(self):
        pass

    def init_action_client(self):
        pass

    def run(self):
        rospy.loginfo("================================")
        rospy.loginfo(" Robinion Leg Kinematics Tester ")
        rospy.loginfo("================================")
        rate = rospy.Rate(10)
        list_l_sole_pose = []
        list_r_sole_pose = []
        
        # Left leg tester
        # Zeros
        pose = Pose()
        pose.position.x = 0.00
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)

        # Front
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)
        
        # Front Left
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH + 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)

        # Back Left
        pose = Pose()
        pose.position.x = -0.05
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH + 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)

        # Back Right
        pose = Pose()
        pose.position.x = -0.05
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH - 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)

        # Front Right
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = self.rb_kinematics.HIP_TO_CROTCH - 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_l_sole_pose.append(pose)
        
        # Right leg tester
        # Zeros
        pose = Pose()
        pose.position.x = 0.00
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)

        # Front
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)
        
        # Front Left
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH + 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)

        # Back Left
        pose = Pose()
        pose.position.x = -0.05
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH + 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)

        # Back Right
        pose = Pose()
        pose.position.x = -0.05
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH - 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)

        # Front Right
        pose = Pose()
        pose.position.x = 0.05
        pose.position.y = -self.rb_kinematics.HIP_TO_CROTCH - 0.05
        pose.position.z = -(self.rb_kinematics.PELVIS_TO_SOLE - 0.035)
        roll = np.radians(0)
        pitch = np.radians(0)
        yaw = np.radians(0)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        list_r_sole_pose.append(pose)

        dt = 0.01
        while not rospy.is_shutdown():
            for i in range(len(list_l_sole_pose)):
                rospy.loginfo("Execute %s", i)
                rospy.loginfo("%s", list_l_sole_pose[i])
                q_l_leg_init = self.rb_joint_states_data.q_l_leg.copy()
                response = self.calc_l_leg_ik_srv(list_l_sole_pose[i])
                q_l_leg_goal = response.q_out
                q_l_leg_traj = self.trajectory_generator.minimum_jerk(q_l_leg_init, q_l_leg_goal, d=1, dt=dt)
                t = 0.0
                for j in range(q_l_leg_traj.shape[0]):
                    # Here we publish to joint
                    self.rb_joint_states_pub.set_l_leg(q_l_leg_traj[j])
                    t += dt
                    # Take sleep
                    rospy.sleep(dt)

            for i in range(len(list_r_sole_pose)):
                rospy.loginfo("Execute %s", i)
                rospy.loginfo("%s", list_r_sole_pose[i])
                q_r_leg_init = self.rb_joint_states_data.q_r_leg.copy()
                response = self.calc_r_leg_ik_srv(list_r_sole_pose[i])
                q_r_leg_goal = response.q_out
                q_r_leg_traj = self.trajectory_generator.minimum_jerk(q_r_leg_init, q_r_leg_goal, d=1, dt=dt)
                t = 0.0
                for j in range(q_r_leg_traj.shape[0]):
                    # Here we publish to joint
                    self.rb_joint_states_pub.set_r_leg(q_r_leg_traj[j])
                    t += dt
                    # Take sleep
                    rospy.sleep(dt)
            
            for i in range(len(list_r_sole_pose)):
                rospy.loginfo("Execute %s", i)
                q_l_leg_init = self.rb_joint_states_data.q_l_leg.copy()
                q_r_leg_init = self.rb_joint_states_data.q_r_leg.copy()
                q_leg_init = np.hstack((q_l_leg_init, q_r_leg_init))
                l_leg_response = self.calc_l_leg_ik_srv(list_l_sole_pose[i])
                r_leg_response = self.calc_r_leg_ik_srv(list_r_sole_pose[i])
                q_leg_goal = np.hstack((l_leg_response.q_out, r_leg_response.q_out))
                q_leg_traj = self.trajectory_generator.minimum_jerk(q_leg_init, q_leg_goal, d=1, dt=dt)
                t = 0.0
                for j in range(q_leg_traj.shape[0]):
                    # Here we publish to joint
                    self.rb_joint_states_pub.set_leg(q_leg_traj[j])
                    t += dt
                    # Take sleep
                    rospy.sleep(dt)

def main():
    node_name = "leg_kinematics_tester_node"
    leg_kinematics_tester = LegKinematicsTester(node_name)
    leg_kinematics_tester.run()

if __name__ == "__main__":
    main()