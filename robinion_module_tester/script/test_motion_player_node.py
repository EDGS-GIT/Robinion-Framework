#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from visualization_msgs.msg import Marker, MarkerArray
from robinion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionFeedback, PlayMotionResult

class MotionPlayerTester():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        self.init_action_client()

    def init_action_client(self):
        self.play_motion_ac = actionlib.SimpleActionClient("/robinion/motion_player/play_motion", PlayMotionAction)
        self.play_motion_ac.wait_for_server()
    
    def run(self):
        rospy.loginfo("===============================")
        rospy.loginfo(" Robinion Motion Player Tester ")
        rospy.loginfo("===============================")
        while not rospy.is_shutdown():
            # Test Zero Pose
            motion_data = PlayMotionGoal()
            motion_data.motion_name = "zero"
            motion_data.movement_time = 5.0
            self.play_motion_ac.send_goal(motion_data)
            self.play_motion_ac.wait_for_result()
            success = self.play_motion_ac.get_result()
            rospy.loginfo("Success %s", success)
            # Test Initialize Pose
            motion_data = PlayMotionGoal()
            motion_data.motion_name = "initialize"
            motion_data.movement_time = 5.0
            self.play_motion_ac.send_goal(motion_data)
            self.play_motion_ac.wait_for_result()
            success = self.play_motion_ac.get_result()
            rospy.loginfo("Success %s", success)
            # Test Standing
            motion_data = PlayMotionGoal()
            motion_data.motion_name = "standing"
            motion_data.movement_time = 5.0
            self.play_motion_ac.send_goal(motion_data)
            self.play_motion_ac.wait_for_result()
            success = self.play_motion_ac.get_result()
            rospy.loginfo("Success %s", success)
            # Test Sitting
            # motion_data = PlayMotionGoal()
            # motion_data.motion_name = "sitting"
            # motion_data.movement_time = 5.0
            # self.play_motion_ac.send_goal(motion_data)
            # self.play_motion_ac.wait_for_result()
            # success = self.play_motion_ac.get_result()
            # rospy.loginfo("Success %s", success)

def main():
    node_name = "motion_player_tester_node"
    motion_player_tester = MotionPlayerTester(node_name)
    motion_player_tester.run()

if __name__ == "__main__":
    main()