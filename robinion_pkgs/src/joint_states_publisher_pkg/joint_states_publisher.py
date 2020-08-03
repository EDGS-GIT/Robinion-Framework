import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class JointStatesPublisher():
    def __init__(self):
        # 0 : head_pitch_joint
        # 1 : head_yaw_joint
        # 2 : l_ankle_pitch_joint
        # 3 : l_ankle_roll_joint
        # 4 : l_elbow_pitch_joint
        # 5 : l_elbow_yaw_joint
        # 6 : l_hip_pitch_joint
        # 7 : l_hip_pitch_passive_1_joint
        # 8 : l_hip_roll_joint
        # 9 : l_hip_yaw_joint
        # 10 : l_knee_passive_1_joint
        # 11 : l_knee_passive_3_joint
        # 12 : l_knee_passive_4_joint
        # 13 : l_shoulder_pitch_joint
        # 14 : l_shoulder_roll_joint
        # 15 : r_ankle_pitch_joint
        # 16 : r_ankle_roll_joint
        # 17 : r_elbow_pitch_joint
        # 18 : r_elbow_yaw_joint
        # 19 : r_hip_pitch_joint
        # 20 : r_hip_pitch_passive_1_joint
        # 21 : r_hip_roll_joint
        # 22 : r_hip_yaw_joint
        # 23 : r_knee_passive_1_joint
        # 24 : r_knee_passive_3_joint
        # 25 : r_knee_passive_4_joint
        # 26 : r_shoulder_pitch_joint
        # 27 : r_shoulder_roll_joint
        # 28 : torso_pitch_joint
        super().__init__()
        self.joint_states = JointState()
        self.init_publisher()

    def test(self):
        print("JointStatesPublisher import success")
    
    def init_publisher(self):
        self.joint_states_pub = rospy.Publisher("/robotis/set_joint_states", JointState, queue_size=1)

    # Head
    def set_head(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['head_yaw_joint',
                                  'head_pitch_joint'] 
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)
    
    # Arm
    def set_l_arm(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_r_arm(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_l_arm_with_torso(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['torso_pitch_joint',
                                  'l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)
    
    def set_r_arm_with_torso(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['torso_pitch_joint',
                                  'r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_arm(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint',
                                  'r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_arm_with_torso(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['torso_pitch',
                                  'l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint',
                                  'r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)
    
    # Leg
    def set_l_leg(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['l_hip_yaw_joint',
                                  'l_hip_roll_joint',
                                  'l_hip_pitch_joint',
                                  'l_ankle_pitch_joint',
                                  'l_ankle_roll_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_r_leg(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['r_hip_yaw_joint',
                                  'r_hip_roll_joint',
                                  'r_hip_pitch_joint',
                                  'r_ankle_pitch_joint',
                                  'r_ankle_roll_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)
    
    def set_leg(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['l_hip_yaw_joint',
                                  'l_hip_roll_joint',
                                  'l_hip_pitch_joint',
                                  'l_ankle_pitch_joint',
                                  'l_ankle_roll_joint',
                                  'r_hip_yaw_joint',
                                  'r_hip_roll_joint',
                                  'r_hip_pitch_joint',
                                  'r_ankle_pitch_joint',
                                  'r_ankle_roll_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    # Body
    def set_full_body_no_torso(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['l_hip_yaw_joint',
                                  'l_hip_roll_joint',
                                  'l_hip_pitch_joint',
                                  'l_ankle_pitch_joint',
                                  'l_ankle_roll_joint',
                                  'r_hip_yaw_joint',
                                  'r_hip_roll_joint',
                                  'r_hip_pitch_joint',
                                  'r_ankle_pitch_joint',
                                  'r_ankle_roll_joint',
                                  'l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint',
                                  'r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint',
                                  'head_yaw_joint',
                                  'head_pitch_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)

    def set_full_body(self, q):
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = ['torso_pitch_joint',
                                  'l_hip_yaw_joint',
                                  'l_hip_roll_joint',
                                  'l_hip_pitch_joint',
                                  'l_ankle_pitch_joint',
                                  'l_ankle_roll_joint',
                                  'r_hip_yaw_joint',
                                  'r_hip_roll_joint',
                                  'r_hip_pitch_joint',
                                  'r_ankle_pitch_joint',
                                  'r_ankle_roll_joint',
                                  'l_shoulder_pitch_joint',
                                  'l_shoulder_roll_joint',
                                  'l_elbow_pitch_joint',
                                  'l_elbow_yaw_joint',
                                  'r_shoulder_pitch_joint',
                                  'r_shoulder_roll_joint',
                                  'r_elbow_pitch_joint',
                                  'r_elbow_yaw_joint',
                                  'head_yaw_joint',
                                  'head_pitch_joint']
        self.joint_states.position = q.tolist()
        self.joint_states_pub.publish(self.joint_states)