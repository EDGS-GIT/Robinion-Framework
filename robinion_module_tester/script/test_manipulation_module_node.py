#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from robinion_msgs.srv import CalcFK, CalcIK
from visualization_msgs.msg import Marker, MarkerArray
from robinion_msgs.msg import MoveArmAction, MoveArmGoal, MoveArmFeedback, MoveArmResult

class ManipulationModuleTester():
    def __init__(self, node_name):
        super().__init__()
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
        self.l_arm_marker_pub = rospy.Publisher("/robinion/manipulation_module/l_arm_marker", Marker, queue_size=1)
        self.r_arm_marker_pub = rospy.Publisher("/robinion/manipulation_module/r_arm_marker", Marker, queue_size=1)

    def init_service_server(self):
        pass

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_l_arm_fk")
        rospy.wait_for_service("/robinion/kinematics_solver/calc_r_arm_fk")
        self.calc_l_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_l_arm_fk", CalcFK)
        self.calc_r_arm_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_r_arm_fk", CalcFK)

    def init_action_server(self):
        pass

    def init_action_client(self):
        self.move_l_arm_ac = actionlib.SimpleActionClient("/robinion/manipulation_module/move_l_arm", MoveArmAction)
        self.move_r_arm_ac = actionlib.SimpleActionClient("/robinion/manipulation_module/move_r_arm", MoveArmAction)
        self.move_l_arm_ac.wait_for_server()
        self.move_r_arm_ac.wait_for_server()

    def run(self):
        lower = np.array([-np.pi*0.5, -np.pi*0.9, -np.pi*0.5, -np.pi*0.9, -np.pi*0.5])
        upper = np.array([np.pi*0.5, np.pi*0.9, np.pi*0.5, np.pi*0.9, np.pi*0.5])
        rospy.loginfo("=====================================")
        rospy.loginfo(" Robinion Manipulation Module Tester ")
        rospy.loginfo("=====================================")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Move left arm")
            q_random = np.random.uniform(low=lower, high=upper)
            result = self.calc_l_arm_fk_srv(q_random)
            # Publish ke RVIZ
            l_arm_marker = Marker()
            l_arm_marker.header.frame_id = "torso_link"
            l_arm_marker.header.stamp = rospy.Time.now()
            l_arm_marker.type = Marker.CUBE
            l_arm_marker.id = 0
            l_arm_marker.pose = result.pose
            l_arm_marker.scale.x = 0.02
            l_arm_marker.scale.y = 0.02
            l_arm_marker.scale.z = 0.02
            l_arm_marker.color.a = 1.0
            l_arm_marker.color.r = 0.0
            l_arm_marker.color.g = 1.0
            l_arm_marker.color.b = 1.0
            self.l_arm_marker_pub.publish(l_arm_marker)
            # Execute motion
            l_arm_data = MoveArmGoal()
            l_arm_data.target_pose = result.pose
            l_arm_data.movement_time = 2.0
            l_arm_data.dt = 0.01
            self.move_l_arm_ac.send_goal(l_arm_data)
            self.move_l_arm_ac.wait_for_result()
            success = self.move_l_arm_ac.get_result()
            rospy.loginfo("Success %s", success)
            rate.sleep()

            rospy.loginfo("Move right arm")
            q_random = np.random.uniform(low=lower, high=upper)
            result = self.calc_r_arm_fk_srv(q_random)
            r_arm_marker = Marker()
            r_arm_marker.header.frame_id = "torso_link"
            r_arm_marker.header.stamp = rospy.Time.now()
            r_arm_marker.type = Marker.CUBE
            r_arm_marker.id = 0
            r_arm_marker.pose = result.pose
            r_arm_marker.scale.x = 0.02
            r_arm_marker.scale.y = 0.02
            r_arm_marker.scale.z = 0.02
            r_arm_marker.color.a = 1.0
            r_arm_marker.color.r = 1.0
            r_arm_marker.color.g = 1.0
            r_arm_marker.color.b = 0.0
            self.r_arm_marker_pub.publish(r_arm_marker)
            r_arm_data = MoveArmGoal()
            r_arm_data.target_pose = result.pose
            r_arm_data.movement_time = 2.0
            r_arm_data.dt = 0.01
            self.move_r_arm_ac.send_goal(r_arm_data)
            self.move_r_arm_ac.wait_for_result()
            success = self.move_r_arm_ac.get_result()
            rospy.loginfo("Success %s", success)
            rate.sleep()

def main():
    node_name = "manipulator_module_tester_node"
    manipulation_module_tester = ManipulationModuleTester(node_name)
    manipulation_module_tester.run()

if __name__ == "__main__":
    main()