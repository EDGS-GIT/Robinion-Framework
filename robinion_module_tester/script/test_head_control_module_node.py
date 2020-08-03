#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from robinion_msgs.srv import CalcFK
from visualization_msgs.msg import Marker
from robinion_msgs.msg import MoveHeadAction, MoveHeadGoal, MoveHeadFeedback, MoveHeadResult

class HeadControlModuleTester():
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
        self.head_marker_pub = rospy.Publisher("/robinion/head_control_module/head_marker", Marker, queue_size=1)

    def init_service_server(self):
        pass

    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_head_fk")
        self.calc_head_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_head_fk", CalcFK)

    def init_action_server(self):
        pass

    def init_action_client(self):
        self.move_head_ac = actionlib.SimpleActionClient("/robinion/head_control_module/move_head", MoveHeadAction)
        self.move_head_ac.wait_for_server()
    
    def run(self):
        lower = np.array([0, -np.pi*0.5, -np.pi*0.5])
        upper = np.array([0, np.pi*0.5, np.pi*0.5])
        rospy.loginfo("=====================================")
        rospy.loginfo(" Robinion Head Control Module Tester ")
        rospy.loginfo("=====================================")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Move head")
            q_random = np.random.uniform(low=lower, high=upper)
            result = self.calc_head_fk_srv(q_random)
            # Publish ke RVIZ
            head_marker = Marker()
            head_marker.header.frame_id = "torso_link"
            head_marker.header.stamp = rospy.Time.now()
            head_marker.type = Marker.CUBE
            head_marker.id = 0
            head_marker.pose = result.pose
            head_marker.scale.x = 0.02
            head_marker.scale.y = 0.02
            head_marker.scale.z = 0.02
            head_marker.color.a = 1.0
            head_marker.color.r = 1.0
            head_marker.color.g = 1.0
            head_marker.color.b = 1.0
            self.head_marker_pub.publish(head_marker)
            # Execute motion
            head_data = MoveHeadGoal()
            head_data.pan = q_random[1]
            head_data.tilt = q_random[2]
            head_data.movement_time = 2.0
            head_data.dt = 0.01
            self.move_head_ac.send_goal(head_data)
            self.move_head_ac.wait_for_result()
            success = self.move_head_ac.get_result()
            rospy.loginfo("Success %s", success)
            rate.sleep()

def main():
    node_name = "head_control_module_tester_node"
    head_control_module_tester = HeadControlModuleTester(node_name)
    head_control_module_tester.run()

if __name__ == "__main__":
    main()