import numpy as np
import rospy
from sensor_msgs.msg import JointState

class JointStatesSubscriber():
    def __init__(self):
        super().__init__()
        self.q_head = np.zeros(2)
        self.q_l_arm = np.zeros(4)
        self.q_r_arm = np.zeros(4)
        self.q_arm = np.zeros(8)
        self.q_arm = np.zeros(8)
        self.q_l_leg = np.zeros(5)
        self.q_r_leg = np.zeros(5)
        self.q_leg = np.zeros(10)
        self.q_leg = np.zeros(10)
        self.q_torso = np.zeros(1)
        self.q_l_arm_with_torso = np.zeros(5)
        self.q_r_arm_with_torso = np.zeros(5)
        self.q_arm_with_torso = np.zeros(9)
        self.q_full_body_no_torso = np.zeros(20)
        self.q_full_body = np.zeros(21)
        # 
        self.init_subscriber()

    def test(self):
        print("JointStatesListener import success")
    
    def init_subscriber(self):
        rospy.Subscriber("/robotis/present_joint_states", JointState, self.joint_states_callback)
    
    def joint_states_callback(self, data):
        # 0 head_pitch_joint
        # 1 head_yaw_joint
        # 2 l_ankle_pitch_joint
        # 3 l_ankle_roll_joint
        # 4 l_elbow_pitch_joint
        # 5 l_elbow_yaw_joint
        # 6 l_hip_pitch_joint
        # 7 l_hip_roll_joint
        # 8 l_hip_yaw_joint
        # 9 l_shoulder_pitch_joint
        # 10 l_shoulder_roll_joint
        # 11 r_ankle_pitch_joint
        # 12 r_ankle_roll_joint
        # 13 r_elbow_pitch_joint
        # 14 r_elbow_yaw_joint
        # 15 r_hip_pitch_joint
        # 16 r_hip_roll_joint
        # 17 r_hip_yaw_joint
        # 18 r_shoulder_pitch_joint
        # 19 r_shoulder_roll_joint
        # 20 torso_pitch_joint

        # head
        self.q_head[0] = data.position[1]
        self.q_head[1] = data.position[0]
        # left arm without torso
        self.q_l_arm[0] = data.position[9]
        self.q_l_arm[1] = data.position[10]
        self.q_l_arm[2] = data.position[4]
        self.q_l_arm[3] = data.position[5]
        # right arm without torso
        self.q_r_arm[0] = data.position[18]
        self.q_r_arm[1] = data.position[19]
        self.q_r_arm[2] = data.position[13]
        self.q_r_arm[3] = data.position[14]
        # left leg
        self.q_l_leg[0] = data.position[8]
        self.q_l_leg[1] = data.position[7]
        self.q_l_leg[2] = data.position[6]
        self.q_l_leg[3] = data.position[2]
        self.q_l_leg[4] = data.position[3]
        # right leg
        self.q_r_leg[0] = data.position[17]
        self.q_r_leg[1] = data.position[16]
        self.q_r_leg[2] = data.position[15]
        self.q_r_leg[3] = data.position[11]
        self.q_r_leg[4] = data.position[12]
        # torso pitch
        self.q_torso[0] = data.position[20]
        # left arm with torso
        self.q_l_arm_with_torso = np.hstack((self.q_torso, self.q_l_arm))
        self.q_r_arm_with_torso = np.hstack((self.q_torso, self.q_r_arm))
        # arm
        self.q_arm = np.hstack((self.q_l_arm, self.q_r_arm))
        self.q_arm_with_torso = np.hstack((self.q_torso, self.q_l_arm, self.q_r_arm))
        # leg
        self.q_leg = np.hstack((self.q_l_leg, self.q_r_leg))
        # body
        self.q_full_body_no_torso = np.hstack((self.q_leg, self.q_arm, self.q_head))
        # body with torso
        self.q_full_body = np.hstack((self.q_torso, self.q_full_body_no_torso))
