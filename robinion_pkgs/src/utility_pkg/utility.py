import PyKDL as kdl
import numpy as np
from geometry_msgs.msg import Pose

class Utility():
    def __init__(self):
        pass

    def test(self):
        print("Import success")

    def kdl_frame_to_ros_pose(self, frame):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = frame.p
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = frame.M.GetQuaternion()
        return pose

    def ros_array_to_kdl_jnt_array(self, ros_array):
        jnt_len = len(ros_array)
        kdl_jnt_array = kdl.JntArray(jnt_len)
        for i in range(jnt_len):
            kdl_jnt_array[i] = ros_array[i]
        return kdl_jnt_array

    def ros_pose_to_kdl_frame(self, ros_pose):
        kdl_frame = kdl.Frame()
        kdl_frame.p = kdl.Vector(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z)
        kdl_frame.M = kdl.Rotation.Quaternion(ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w)
        return kdl_frame

    def kdl_jnt_array_to_ros_array(self, kdl_jnt_array):
        jnt_len = kdl_jnt_array.rows()
        ros_array = []
        for i in range(jnt_len):
            ros_array.append(kdl_jnt_array[i])
        return ros_array

    def calc_error(self, target_frame, current_frame):
        diff_frame = target_frame.Inverse() * current_frame
        [dx, dy, dz] = diff_frame.p
        [drz, dry, drx] = diff_frame.M.GetEulerZYX()
        error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
        return error