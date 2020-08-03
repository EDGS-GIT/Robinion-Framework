import numpy as np
from tf.transformations import euler_from_quaternion
# import Py3KDL as kdl
import PyKDL as kdl
import rospy

class LegKinematics():
    def __init__(self):
        super().__init__()
        # Link configuration
        self.crotch_to_hip_yaw = 0.055
        self.E = 0.050 # Jarak hip yaw ke hip roll pitch
        self.A = 0.2215 # Panjang link upper leg
        self.B = 0.2215 # Panjang link lower leg
        self.F = 0.053 # Jarak ankle ke sole
        # self.init_kdl_chain()
    
    def RX(self, alpha):
        return np.array([[1, 0, 0], 
                         [0, np.cos(alpha), -np.sin(alpha)], 
                         [0, np.sin(alpha), np.cos(alpha)]])   

    def RY(self, delta):
        return np.array([[np.cos(delta), 0, np.sin(delta)], 
                         [0, 1, 0], 
                         [-np.sin(delta), 0, np.cos(delta)]])

    def RZ(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0], 
                         [np.sin(theta), np.cos(theta), 0], 
                         [0, 0, 1]])

    def TF(self, rot_axis=None, q=0, dx=0, dy=0, dz=0):
        if rot_axis == 'x':
            R = self.RX(q)
        elif rot_axis == 'y':
            R = self.RY(q)
        elif rot_axis == 'z':
            R = self.RZ(q)
        elif rot_axis == None:
            R = np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])
        
        T = np.array([[R[0,0], R[0,1], R[0,2], dx],
                      [R[1,0], R[1,1], R[1,2], dy],
                      [R[2,0], R[2,1], R[2,2], dz],
                      [0, 0, 0, 1]])
        return T
    
    def init_kdl_chain(self):
        self.l_leg_chain = kdl.Chain()
        self.r_leg_chain = kdl.Chain()

        # Pelvis = Base as origin
        pelvis_fixed = kdl.Joint()
        pelvis_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, 0.0))
        pelvis_segment = kdl.Segment(pelvis_fixed, pelvis_frame)
        self.l_leg_chain.addSegment(pelvis_segment)
        self.r_leg_chain.addSegment(pelvis_segment)
        
        # Upper left hip and right hip
        crotch_to_hip_yaw_fixed = kdl.Joint()
        upper_l_hip_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, self.crotch_to_hip_yaw, 0.0))
        upper_r_hip_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, -self.crotch_to_hip_yaw, 0.0))
        upper_l_hip_segment = kdl.Segment(crotch_to_hip_yaw_fixed, upper_l_hip_frame)
        upper_r_hip_segment = kdl.Segment(crotch_to_hip_yaw_fixed, upper_r_hip_frame)
        self.l_leg_chain.addSegment(upper_l_hip_segment)
        self.r_leg_chain.addSegment(upper_r_hip_segment)
        
        # Hip yaw
        hip_yaw_joint = kdl.Joint(kdl.Joint.RotZ)
        hip_yaw_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, 0.0))
        hip_yaw_segment = kdl.Segment(hip_yaw_joint, hip_yaw_frame)
        self.l_leg_chain.addSegment(hip_yaw_segment)
        self.r_leg_chain.addSegment(hip_yaw_segment)
        
        # Hip roll
        hip_roll_joint = kdl.Joint(kdl.Joint.RotX)
        hip_roll_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, -self.E))
        hip_roll_segment = kdl.Segment(hip_roll_joint, hip_roll_frame)
        self.l_leg_chain.addSegment(hip_roll_segment)
        self.r_leg_chain.addSegment(hip_roll_segment)

        # Hip pitch
        hip_pitch_joint = kdl.Joint(kdl.Joint.RotY)
        hip_pitch_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, 0.0))
        hip_pitch_segment = kdl.Segment(hip_pitch_joint, hip_pitch_frame)
        self.l_leg_chain.addSegment(hip_pitch_segment)
        self.r_leg_chain.addSegment(hip_pitch_segment)
        
        # Knee
        knee_joint = kdl.Joint(kdl.Joint.RotY)
        knee_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, -self.A))
        knee_segment = kdl.Segment(knee_joint, knee_frame)
        self.l_leg_chain.addSegment(knee_segment)
        self.r_leg_chain.addSegment(knee_segment)
        
        # Ankle pitch
        ankle_pitch_joint = kdl.Joint(kdl.Joint.RotY)
        ankle_pitch_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, -self.B))
        ankle_pitch_segment = kdl.Segment(ankle_pitch_joint, ankle_pitch_frame)
        self.l_leg_chain.addSegment(ankle_pitch_segment)
        self.r_leg_chain.addSegment(ankle_pitch_segment)
        
        # Ankle roll
        ankle_roll_joint = kdl.Joint(kdl.Joint.RotX)
        ankle_roll_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, 0.0))
        ankle_roll_segment = kdl.Segment(ankle_roll_joint, ankle_roll_frame)
        self.l_leg_chain.addSegment(ankle_roll_segment)
        self.r_leg_chain.addSegment(ankle_roll_segment)

        # Sole
        ankle_to_sole_fixed = kdl.Joint()
        sole_frame = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.0, 0.0, -self.F))
        sole_segment = kdl.Segment(ankle_to_sole_fixed, sole_frame)
        self.l_leg_chain.addSegment(sole_segment)
        self.r_leg_chain.addSegment(sole_segment)
        
        self.l_leg_fk = kdl.ChainFkSolverPos_recursive(self.l_leg_chain)
        self.r_leg_fk = kdl.ChainFkSolverPos_recursive(self.r_leg_chain)
    
    def get_full_leg_joints(self, q):
        # input berupa list joint
        # output berupa kdl jnt array
        q_kdl_jnt_array = kdl.JntArray(6)
        q_kdl_jnt_array[0] = q[0] # hip yaw
        q_kdl_jnt_array[1] = q[1] # hip roll
        q_kdl_jnt_array[2] = q[2] # hip pitch

        alpha = -q[2]
        gamma = np.pi/2 + q[3]
        beta = np.pi - (np.pi/2) - gamma
        qc = np.pi - alpha - beta
        q_kdl_jnt_array[3] = np.pi - qc

        q_kdl_jnt_array[4] = q[3] # ankle pitch
        q_kdl_jnt_array[5] = q[4] # ankle roll
        total = qc + q_kdl_jnt_array[3]

        return q_kdl_jnt_array
    
    def fk_leg(self, q_in, leg=0):
        base = self.TF()
        if leg == 0:
            hip_yaw_from_base = self.TF(rot_axis='z', q=q_in[0], dy=self.crotch_to_hip_yaw)
        else:
            hip_yaw_from_base = self.TF(rot_axis='z', q=q_in[0], dy=-self.crotch_to_hip_yaw)
        hip_yaw = base.dot(hip_yaw_from_base)
        hip_roll_from_hip_yaw = self.TF(rot_axis='x', q=q_in[1], dz=-self.E)
        hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw)
        hip_pitch_from_hip_roll = self.TF(rot_axis='y', q=q_in[2])
        hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll)
        hip = hip_pitch # hip frame
        knee_from_hip = self.TF(rot_axis='y', q=q_in[3], dz=-self.A)
        knee = hip.dot(knee_from_hip) # knee frame
        ankle_pitch_from_knee = self.TF(rot_axis='y', q=q_in[4], dz=-self.B)
        ankle_pitch = knee.dot(ankle_pitch_from_knee)
        ankle_roll_from_ankle_pitch = self.TF(rot_axis='x', q=q_in[5])
        ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch)
        ankle = ankle_roll # ankle frame
        sole_from_ankle = self.TF(dz=-self.F)
        sole = ankle.dot(sole_from_ankle) # sole frame
        eef_frame_kdl = kdl.Frame(kdl.Rotation(sole[0,0],sole[0,1],sole[0,2],
                                               sole[1,0],sole[1,1],sole[1,2],
                                               sole[2,0],sole[2,1],sole[2,2]),
                                  kdl.Vector(sole[0,3], sole[1,3], sole[2,3]))
        return eef_frame_kdl

    def fk_l_leg(self, q_l_leg_kdl):
        return self.fk_leg(q_l_leg_kdl, leg=0)
        
    def fk_r_leg(self, q_r_leg_kdl):
        return self.fk_leg(q_r_leg_kdl, leg=1)

    # 0 left
    # 1 right
    def leg_ik(self, pose, leg=0):
        if leg == 0:
            self.D = self.crotch_to_hip_yaw
        else:
            self.D = -self.crotch_to_hip_yaw

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        (roll, pitch, yaw) = euler_from_quaternion ([pose.orientation.x, pose.orientation.y,
                                                     pose.orientation.z, pose.orientation.w])
        
        x_from_upper_hip = (x - 0)
        y_from_upper_hip = (y - self.D)
        z_from_upper_hip = (z + self.E + self.F)

        xa = x_from_upper_hip
        ya_1 = xa * np.tan(yaw)
        xb_1 = xa / np.cos(yaw)
        beta = np.pi/2 - yaw
        ya_2 = y_from_upper_hip - ya_1
        yb = ya_2 * np.sin(beta)
        xb_2 = yb / np.tan(beta)
        xb = xb_1 + xb_2

        x_from_hip_yaw = xb
        y_from_hip_yaw = yb
        z_from_hip_yaw = z_from_upper_hip

        C = np.sqrt(x_from_hip_yaw**2 + y_from_hip_yaw**2 + z_from_hip_yaw**2)
        qx = np.arcsin(x_from_hip_yaw / C)
        qb = np.arccos((C/2) / self.A)
        
        qy = np.arctan2(y_from_hip_yaw,np.sign(z_from_hip_yaw)*z_from_hip_yaw)
        qc = np.pi - (qb*2)
        qz = np.arcsin(abs(z_from_hip_yaw)/C)
        qa = qb

        q_hip_yaw = yaw
        q_hip_roll = qy
        q_hip_pitch = -(qx + qb)
        q_ankle_roll = -qy
        q_knee = np.pi - qc # useless
        if xb >= 0:
            q_ankle_pitch = -(np.pi/2 - (np.pi - (qz + qa)))
        else:
            q_ankle_pitch = (qz-qa-(np.pi/2))
        
        return [q_hip_yaw, q_hip_roll, q_hip_pitch, q_ankle_pitch, q_ankle_roll]
    
    def calc_l_leg_ik(self, pose):
        return self.leg_ik(pose, leg=0)
    
    def calc_r_leg_ik(self, pose):
        return self.leg_ik(pose, leg=1)
    