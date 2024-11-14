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
        # Control Variance
        self.label_position_variance = 0.0001
        # Label Variance
        self.control_position_variance = 0.0225
        self.control_orientation_variance = 0.0609
        # Covariance
        self.conv_factor = 0.000001
        # Initialize Value for P
        self.init_label_position_variance = 100

    # Get State and P
    def getMap(self):
        return self.state, self.P
    
    # Get robot current pose
    def getPose(self):
        xr = self.state[0,0]
        yr = self.state[1,0]
        theta = self.state[2,0]
        return [xr, yr, theta]
    
    # Add new Label to state and P
    def add_label_to_state_and_covariance(self, marker):
        marker_id = marker['id']
        marker_pose = marker['pose']
        new_state = np.vstack((self.state, marker_pose))
        
        n = self.P.shape[0]
        new_P = np.zeros((n + 4, n + 4))
        new_P[:n, :n] = self.P 
        new_P[n, n] = self.init_label_position_variance
        new_P[n + 1, n + 1] = self.init_label_position_variance
        new_P[n + 2, n + 2] = self.init_label_position_variance
        new_P[n + 3, n + 3] = 0.0

        self.state = new_state
        self.P = new_P
        self.marker_indices[marker_id] = n

    # Set F Matrix for Prediction
    def setF(self):
        n = len(self.state)
        self.F = np.eye(n)
    
    # Set R Measurement Noise Matrix for Update
    def setR(self, num_markers):
        # Load Measured R for each label
        package_share_directory = get_package_share_directory('hw3_map')
        r_matrix_path = os.path.join(package_share_directory, 'resource', 'R_matrix.npy')
        R = np.load(r_matrix_path)
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
        row_indices = []
        col_indices = []
        data_values = []
        for i, marker_id in enumerate(marker_ids):
            index = self.marker_indices[marker_id]
            row_start = i * 4
            col_start = index

            H_i_nonzero = np.nonzero(H_low)
            H_i_row_indices = H_i_nonzero[0]
            H_i_col_indices = H_i_nonzero[1]
            H_i_data_values = H_low[H_i_row_indices, H_i_col_indices]

            row_indices.extend(row_start + H_i_row_indices)
            col_indices.extend(col_start + H_i_col_indices)
            data_values.extend(H_i_data_values)

        H_sparse = coo_matrix((data_values, (row_indices, col_indices)), shape=(num_markers * 4, n))
        H_sparse = H_sparse.tocsr()
        self.H = H_sparse

    # Set Q System Model Error Matrix for Prediction
    def setQ(self):
        n = len(self.state)
        self.Q = np.zeros((n, n))
        # Control Variance
        self.Q[0, 0] = self.control_position_variance
        self.Q[1, 1] = self.control_position_variance
        self.Q[2, 2] = self.control_orientation_variance
        # Label Variance
        for i in range(3, n, 4): 
            self.Q[i, i] = self.label_position_variance        
            self.Q[i + 1, i + 1] = self.label_position_variance  
            self.Q[i + 2, i + 2] = self.label_position_variance  
            self.Q[i + 3, i + 3] = 0  

            # Covariance
            self.Q[0, i] = self.conv_factor
            self.Q[i, 0] = self.conv_factor
            self.Q[1, i + 1] = self.conv_factor
            self.Q[i + 1, 1] = self.conv_factor
    
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
        self.P = self.F @ self.P @ self.F.T + self.Q

    def KalmanUpdate(self, z, marker_ids):
        # Set H, R
        self.setH(marker_ids)
        self.setR(len(marker_ids))

        # Compute S
        self.S = self.H @ self.P @ self.H.transpose() + self.R

        # Compute K
        self.K = self.P @ self.H.transpose() @ np.linalg.pinv(self.S)

        # Compute update state
        self.state = self.state + self.K @ (z - self.H @ self.state)

        # Compute update P
        KH = self.K @ self.H
        I = np.eye(KH.shape[0])
        self.P = (I - KH) @ self.P

    

    