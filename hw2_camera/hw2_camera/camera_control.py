import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Gain
KA = 1.7
KP = 0.8
KB = -0.44

P_THRESHOLD = 0.3

class Camera_control():
    def __init__(self):
        self.ka = KA
        self.kp = KP
        self.kb = KB
        self.pthreshold = P_THRESHOLD


    def control(self, current, target):

        # Current robot position and orientation in world frame
        x_r, y_r, theta_r = current[0], current[1], current[2]

        # Target position in world frame
        x_w, y_w, theta_w = target[0], target[1], target[2]

        # Control
        dx = x_w - x_r
        dy = y_w - y_r
        p = np.sqrt(dx**2 + dy**2)
        a = math.atan2(math.sin(theta_w-theta_r),math.cos(theta_w-theta_r))

        # Decide whether to move forward or backward
        if abs(a) == np.pi:
            a = 0
            p = -p

        v = self.kp * p
        gamma = self.ka * a 
        
        print(f"vx: {v}, vy: {0}, wz: {gamma}")
        t_translation = abs(p / v) if v != 0 else 0  
        t_rotation = abs(a / gamma) if gamma != 0 else 0  
        t_total = t_translation + t_rotation
        return [v, 0, gamma, t_total]
    
    def cameraToRobot(self,trans):
        x = trans[2]
        y = -trans[0] - 0.01
        return np.array([x,y])
    
    def computeObservedPose(self, trans, ori, label_pos):
        rotation = R.from_quat(ori)
        roll_pitch_yaw = rotation.as_euler('xyz', degrees=True)
        dywa_r = math.radians(roll_pitch_yaw.flatten()[0])
        dywa_w = label_pos[2] - math.pi - dywa_r
        # orientation of image from robot
        theta_r = np.arctan2(np.sin(dywa_w), np.cos(dywa_w))
        # translation of image from robot
        trans_r = self.cameraToRobot(trans)

        pose_l_w = np.array([[label_pos[0]], [label_pos[1]]])
        pose_l_r = np.array([[trans_r[0]], [trans_r[1]]])
        R_robot_world = np.array([
            [np.cos(theta_r), -np.sin(theta_r)],
            [np.sin(theta_r), np.cos(theta_r)]
        ])
        pos_robot_w = pose_l_w - np.dot(R_robot_world, pose_l_r)
        x = pos_robot_w.flatten()[0]
        y = pos_robot_w.flatten()[1]
        return [x,y,theta_r]
    
    def updatePose(self,motion,current):
        v = motion[0]
        gamma = motion[2]
        t = motion[3]
        matrix_v_g = np.array([[v],[gamma]])
        matrix_rotate = np.array([[math.cos(current[2]), 0], 
                                  [math.sin(current[2]), 0], 
                                  [0,1]])
        res = np.dot(matrix_rotate, matrix_v_g)
        res = res.T.flatten()
        vx_w = res[0]
        vy_w = res[1]
        wz = res[2]
        x_w = current[0] + vx_w * t
        y_w = current[1] + vy_w * t
        theta_w = current[2] + wz * t
        theta_w = (theta_w + math.pi) % (2 * math.pi) - math.pi
        return [x_w, y_w, theta_w]