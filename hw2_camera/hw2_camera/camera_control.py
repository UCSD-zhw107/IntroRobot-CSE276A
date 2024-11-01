import math
import numpy as np
from scipy.spatial.transform import Rotation as R


"""
KP=1.2
"""
"""
KA=1.1
KP=1.3
"""

# Gain
KA = 1.2
KP = 0.57
KB = 1.2

P_THRESHOLD = 0.3

class Camera_control():
    def __init__(self):
        self.ka = KA
        self.kp = KP
        self.kb = KB
        self.pthreshold = P_THRESHOLD


    def control(self, current, target,dt, state):

        # Current robot position and orientation in world frame
        x_r, y_r, theta_r = current[0], current[1], current[2]

        # Target position in world frame
        x_w, y_w, theta_w = target[0], target[1], target[2]

        # Control
        dx = x_w - x_r
        dy = y_w - y_r
        p = np.sqrt(dx**2 + dy**2)
        a = math.atan2(math.sin(math.atan2(dy,dx)-theta_r),math.cos(math.atan2(dy,dx)-theta_r))
        b = np.arctan2(np.sin(theta_w - theta_r), np.cos(theta_w - theta_r))
        
        # Decide whether to move forward or backward
        if abs(a) == np.pi:
            a = 0
            p = -p

        v = self.kp * p
        gamma = 0
        if state == 'rotation':
            gamma = self.kb * b
        else:
            gamma = self.ka * a if p > self.pthreshold else self.kb * b
        #print(f"vx: {v}, vy: {0}, wz: {gamma}")
        t_translation = abs(p / v) if v != 0 else 0  
        t_rotation = abs(a / gamma) if gamma != 0 else 0  
        t_total = t_translation + t_rotation
        return [v, 0, gamma, dt]
    
    def cameraToRobot(self,trans):
        x = trans[2]
        y = -trans[0] + 0.01
        return np.array([x,y])
    
    '''def computeObservedPose(self, trans, ori, label_pos):
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
        x = pos_robot_w.T.flatten()[0]
        y = pos_robot_w.T.flatten()[1]
        return [x,y,theta_r]'''
    
    def computeObservedPose(self, pose,ori,x_w):
        rotation = R.from_quat(ori)
        roll = rotation.as_euler('xyz', degrees=False)[0]
        label_ori = np.arctan2(np.sin(roll-math.pi), np.cos(roll - math.pi))
        label_pose = self.cameraToRobot(pose)
        T_r_l = np.array([
            [np.cos(label_ori), -np.sin(label_ori), label_pose[0]],
            [np.sin(label_ori), np.cos(label_ori), label_pose[1]],
            [0,0,1]
        ])

        T_l_w = np.array([
            [np.cos(x_w[2]), -np.sin(x_w[2]), x_w[0]],
            [np.sin(x_w[2]), np.cos(x_w[2]), x_w[1]],
            [0,0,1]
        ])

        A_inv = np.linalg.inv(T_r_l)
        res = np.dot(T_l_w, A_inv)
        theta = np.arctan2(res[1,0], res[0,0])
        x = res[0,2]
        y = res[1,2]
        return [x,y,theta]

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
    

    def pose_error(self,current, target):
    
        x1, y1, theta1 = current[0], current[1], current[2]
        x2, y2, theta2 = target[0], target[1], target[2]

        distance_error = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        
        angle_error = math.atan2(math.sin(theta2 - theta1), math.cos(theta2 - theta1))

        return distance_error, angle_error