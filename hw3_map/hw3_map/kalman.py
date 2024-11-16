#!/usr/bin/env python3
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.sparse import coo_matrix
from scipy.linalg import block_diag
from ament_index_python.packages import get_package_share_directory

class KalmanFilter():
    def __init__(self):
        self.state = np.zeros((3, 1))
        self.P = np.zeros((3, 3))
        self.H = None
        self.Q = None
        self.R = None
        self.K = None
        self.S = None
        self.F = None
        self.marker_indices = {}
        # Variance for Q
        # Label Variance
        self.label_position_variance = 0.5
        # Control Variance
        self.control_position_variance_x = 0.0525
        self.control_position_variance_y = 0.1
        self.control_orientation_variance = 0.209
        # Covariance
        self.conv_factor = 0.00001
        # Initialize Value for P
        self.init_label_position_variance = 3.0
        # R Matrix path
        package_share_directory = get_package_share_directory('hw3_map')
        self.r_matrix_path = os.path.join(package_share_directory, 'resource', 'R_matrix.npy')

    # Get State and P
    def getMap(self):
        return self.state, self.P
    
    # Get robot current pose
    def getPose(self):
        xr = self.state[0,0]
        yr = self.state[1,0]
        theta = self.state[2,0]
        return np.array([xr, yr, theta])
    
    def setPose(self, pose):
        self.state[0,0] = pose[0]
        self.state[1,0] = pose[1]
        self.state[2,0] = pose[2]
    
    # Add new Label to state and P
    def add_labels_to_state_and_covariance(self, labels):
        for marker in labels:
            marker_id = marker['id']
            marker_pose = marker['pose']
            if marker_id in self.marker_indices:
                continue
            self.state = np.vstack((self.state, marker_pose))
            n = self.P.shape[0]
            new_P = np.zeros((n + 4, n + 4))
            new_P[:n, :n] = self.P  
            new_P[n, n] = self.init_label_position_variance
            new_P[n + 1, n + 1] = self.init_label_position_variance
            new_P[n + 2, n + 2] = self.init_label_position_variance
            new_P[n + 3, n + 3] = 0.0  

            self.P = new_P
            self.marker_indices[marker_id] = n

    # Set F Matrix for Prediction
    def setF(self):
        n = len(self.state)
        self.F = np.eye(n)
    
    # Set R Measurement Noise Matrix for Update
    def setR(self, num_markers):
        # Load Measured R for each label
        #R = np.load(self.r_matrix_path)
        R = np.array([
            [0.1, 0.0, 0.0, 0.0],
            [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        self.R = block_diag(*[R] * num_markers)


    # Set H Observation Matrix for Update
    def setH(self, marker_ids):
        # Construct H Low
        xr = self.state[0,0]
        yr = self.state[1,0]
        theta = self.state[2,0]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        rot_robot_map = np.array([
            [-sin_theta, 0.0, cos_theta],
			[-cos_theta, 0.0, sin_theta],
			[0.0,-1.0,0.0]]
        ).T
        trans_robot_map = np.array([xr, yr, 0.])
        trans_robot_map = -np.dot(rot_robot_map, trans_robot_map)
        H_low = np.eye(4)
        H_low[:3, :3] = rot_robot_map
        H_low[:3, 3] = trans_robot_map

        # Construct H
        n = self.state.shape[0]
        num_markers = len(marker_ids)
        H = np.zeros((num_markers * 4, n))  # Dense matrix for H

        for i, marker_id in enumerate(marker_ids):
            index = self.marker_indices[marker_id]
            row_start = i * 4
            col_start = index

            # Assign values from H_low to the correct block in H
            H[row_start:row_start + 4, col_start:col_start + 4] = H_low

        self.H = H

    # Set Q System Model Error Matrix for Prediction
    def setQ(self):
        n = len(self.state)
        self.Q = np.zeros((n, n))
        # Control Variance
        self.Q[0, 0] = self.control_position_variance_x
        self.Q[1, 1] = self.control_position_variance_y
        self.Q[2, 2] = self.control_orientation_variance
        # Label Variance
        for i in range(3, n, 4): 
            self.Q[i, i] = 0.0        
            self.Q[i + 1, i + 1] = 0.0  
            self.Q[i + 2, i + 2] = 0.0  
            self.Q[i + 3, i + 3] = 0.0
    
    # Kalman Prediction
    def kalmanPredict(self, move_update):
        # Set F, Q
        self.setF()
        self.setQ()

        # Predict State after movement
        self.state = np.dot(self.F, self.state)
        self.state[0, 0] += move_update[0]
        self.state[1, 0] += move_update[1]
        self.state[2, 0] += move_update[2]

        # Predict P after movemet
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def kalmanUpdate(self, z, marker_ids):
        # Set H, R
        self.setH(marker_ids)
        self.setR(len(marker_ids))
        #print(f'Z: {z}')
        #print(f'HS: {np.dot(self.H,self.state)}')

        # Compute S
        self.S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # Compute K
        self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.pinv(self.S))

        # Compute update state
        error = z - np.dot(self.H, self.state)
        self.state = self.state + np.dot(self.K, error)

        # Compute update P
        KH = np.dot(self.K, self.H)
        prior = np.eye(self.P.shape[0]) - KH
        self.P = np.dot(prior, self.P)

    def displayMap(self):
        robot_pose = self.getPose()
        print('+++++++++++++++++++++++++')
        print(f'Robot Pose: {robot_pose}')
        print('+++++++++++++++++++++++++')
        for i in self.marker_indices.keys():
            print(f'Tag ID: {i}')
            entry = self.marker_indices[i]
            tag_x = self.state[entry]
            tag_y = self.state[entry + 1]
            tag_pose = [tag_x, tag_y]
            print(f'Tag Pose: {tag_pose}')
            print('+++++++++++++++++++++++++')


    

    