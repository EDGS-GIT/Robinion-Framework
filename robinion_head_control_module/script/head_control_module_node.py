#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from robinion_msgs.srv import CalcFK
from visualization_msgs.msg import Marker, MarkerArray
from robinion_msgs.msg import MoveHeadAction, MoveHeadFeedback, MoveHeadResult
from trajectory_generator_pkg.trajectory_generator import TrajectoryGenerator
from joint_states_subscriber_pkg.joint_states_subscriber import JointStatesSubscriber
from joint_states_publisher_pkg.joint_states_publisher import JointStatesPublisher

class HeadControlModule():
    def __init__(self, node_name):
        super().__init__()
        rospy.init_node(node_name)
        self.q_head_goal = np.zeros(2)
        self.head_is_moving = False
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
        rospy.Subscriber("/robinion/head_control_module/set_head_joint", JointState, self.set_head_joint_callback)

    def init_publisher(self):
        pass

    def init_service_server(self):
        pass
    
    def init_service_client(self):
        rospy.wait_for_service("/robinion/kinematics_solver/calc_head_fk")
        self.calc_head_fk_srv = rospy.ServiceProxy("/robinion/kinematics_solver/calc_head_fk", CalcFK)

    def init_action_server(self):
        self.move_head_as = actionlib.SimpleActionServer("/robinion/head_control_module/move_head", 
                                                          MoveHeadAction, execute_cb=self.move_head_action_callback,
                                                          auto_start=False)
        self.move_head_as.start()

    def init_action_client(self):
        pass

    def set_head_joint_callback(self, data):
        # Execute when head is not move, prevent conflict with action server
        if not self.head_is_moving:
            for i in range(len(data.name)):
                if data.name[i] == 'pan':
                    self.q_head_goal[0] = data.position[i]
                elif data.name[i] == 'tilt':
                    self.q_head_goal[1] = data.position[i]
            self.rb_joint_states_pub.set_head(self.q_head_goal)
    
    def move_head_action_callback(self, data):
        self.head_is_moving = True
        feedback = MoveHeadFeedback()
        result = MoveHeadResult()
        rospy.loginfo("Execute motion")
        self.q_head_init = self.rb_joint_states_data.q_head.copy()
        self.q_head_goal[0] = data.pan 
        self.q_head_goal[1] = data.tilt
        success = True
        
        head_traj = self.trajectory_generator.minimum_jerk(self.q_head_init, 
                                                           self.q_head_goal,
                                                           d=data.movement_time,
                                                           dt=data.dt)
        t = 0.0

        for i in range(head_traj.shape[0]):
            if self.move_head_as.is_preempt_requested():
                success = False
                break
            # Here we publish the feedback of head pose
            response = self.calc_head_fk_srv(head_traj[i])
            feedback.current_pose = response.pose
            feedback.current_time = t
            self.move_head_as.publish_feedback(feedback)
            # Here we publish to joint 
            self.rb_joint_states_pub.set_head(head_traj[i])
            t += data.dt
            # Take sleep
            rospy.sleep(data.dt)

        result.success = success
        self.move_head_as.set_succeeded(result)
        self.head_is_moving = False

    def run(self):
        rospy.loginfo("==============================")
        rospy.loginfo(" Robinion Head Control Module ")
        rospy.loginfo("==============================")
        rospy.spin()

def main():
    node_name = "head_control_module_node"
    head_control_module = HeadControlModule(node_name)
    head_control_module.run()

if __name__ == "__main__":
    main()