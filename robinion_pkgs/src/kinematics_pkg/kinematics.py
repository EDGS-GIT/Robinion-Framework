import rospy
import numpy

class Kinematics():
    def __init__(self):
        super().__init__()
        # Robinion kinematics
        self.HIP_TO_CROTCH = 0.055
        self.UPPER_HIP = 0.050
        self.HIP_TO_KNEE = 0.2215
        self.KNEE_TO_ANKLE = 0.2215
        self.ANKLE_TO_SOLE = 0.053
        self.PELVIS_TO_SOLE = self.UPPER_HIP + self.HIP_TO_KNEE + self.KNEE_TO_ANKLE + self.ANKLE_TO_SOLE
    
    def test(self):
        print("Import kinematics success")