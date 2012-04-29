from numpy import *

class accelerometer:
    def __init__(self):
        self.dcm = zeros([3,3])

    def compute_dcm(self,phi,theta,psi):
        self.dcm = array(
            [[math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi)],
            [math.cos(theta)*math.cos(psi), math.sin(phi)*math.sin(theta)*math.cos(psi)+math.cos(phi)*math.sin(psi), math.cos(phi)*math.sin(theta)*math.cos(psi)-math.sin(phi)*math.sin(psi)],
            [-math.sin(theta), math.sin(phi)*math.cos(theta), math.cos(phi)*math.cos(theta)]]
            )

    def tps_to_body():
        compute_dcm
