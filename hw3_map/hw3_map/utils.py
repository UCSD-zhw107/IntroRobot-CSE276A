#!/usr/bin/env python3
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log
import numpy as np
from scipy.spatial.transform import Rotation as R

def projectApriltag(rot_camera_apriltag, trans_camera_apriltag):
    """
    Project R and t from camera to apriltag into 2D

    Parameters:
    rot_camera_apriltag: R from camera to apriltag
    trans_camera_apriltag: t from camera to apriltag

    Return
    R and t from camera to apriltag into 2D
    """
    trans_camera_apriltag_2d = trans_camera_apriltag
    trans_camera_apriltag_2d[1] = 0.0

    rot_camera_apriltag_2d = np.array([
        [rot_camera_apriltag[0,0], 0.0, rot_camera_apriltag[2,0]],
        [0.0, 1.0, 0.0],
        [rot_camera_apriltag[2,0], 0.0, rot_camera_apriltag[0,0]],
    ])
    return rot_camera_apriltag_2d, trans_camera_apriltag_2d

def combine_transformations(R1, t1, R2, t2):
    """
    Combine two transformations given by rotation matrices and translation vectors.
    
    Parameters:
    R1 (numpy.ndarray): First 3x3 rotation matrix.
    t1 (numpy.ndarray): First translation vector, shape (3,).
    R2 (numpy.ndarray): Second 3x3 rotation matrix.
    t2 (numpy.ndarray): Second translation vector, shape (3,).
    
    Returns:
    R_combined (numpy.ndarray): Combined 3x3 rotation matrix.
    t_combined (numpy.ndarray): Combined translation vector, shape (3,).
    """
    # Ensure the input matrices and vectors have correct shapes
    assert R1.shape == (3, 3), "R1 must be a 3x3 matrix"
    assert t1.shape == (3,), "t1 must be a 3-element vector"
    assert R2.shape == (3, 3), "R2 must be a 3x3 matrix"
    assert t2.shape == (3,), "t2 must be a 3-element vector"
    
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = t1

    T2 = np.eye(4)
    T2[:3, :3] = R2
    T2[:3, 3] = t2
    
    return np.dot(T1, T2)


def findInverse(rot, trans):
    rot_inv = rot.T
    trans_inv = -np.dot(rot_inv, trans)
    return rot_inv, trans_inv


def findRobotMap(quat_camera_apriltag, trans_camera_apriltag, pose_map_apriltag):
    """
    Compute Robot Pose in Map based on

    Parameters:
    quat_camera_apriltag: Quat from camera to apriltag
    trans_camera_apriltag: t from camera to apriltag
    pose_map_apriltag: Pose of apriltag in map frame

    Return:
    Pose of Robot in map frame
    """
    # Compute R and t from apriltag to camera
    rot_camera_apriltag = R.from_quat(quat_camera_apriltag).as_matrix()
    #rot_camera_apriltag, trans_camera_apriltag = projectApriltag(rot_camera_apriltag,trans_camera_apriltag)
    rot_apriltag_camera, trans_apriltag_camera = findInverse(rot_camera_apriltag, trans_camera_apriltag)

    # Compute R and t from map to apriltag
    rot_map_apriltag = R.from_quat(pose_map_apriltag[3:]).as_matrix()
    trans_map_apriltag = np.asarray(pose_map_apriltag[:3])

    # Compute R and t from map to camera
    T_map_camera = combine_transformations(
    rot_map_apriltag, trans_map_apriltag,
    rot_apriltag_camera, trans_apriltag_camera,
    )
    rot_map_camera, trans_map_camera = T_map_camera[:3, :3], T_map_camera[:3, 3]
    angle = math.atan2(rot_map_camera[1][2], rot_map_camera[0][2])
    return np.array([trans_map_camera[0], trans_map_camera[1], angle])


def findAprilTagMap(quat_camera_apriltag, trans_camera_apriltag, pose_map_camera):
    """
    Compute AprilTag Pose in Map based on

    Parameters:
    quat_camera_apriltag: Quat from camera to apriltag
    trans_camera_apriltag: t from camera to apriltag
    pose_map_camera: Pose of camera in map frame

    Return:
    Pose of Apriltag in map frame
    """
    # Compute R and t from apriltag to camera
    rot_camera_apriltag = R.from_quat(quat_camera_apriltag).as_matrix()
    #rot_camera_apriltag, trans_camera_apriltag = projectApriltag(rot_camera_apriltag,trans_camera_apriltag)

    # Compute R and t from map to camera
    rot_map_camera = np.array([
            [-np.sin(pose_map_camera[2]), 0.0, np.cos(pose_map_camera[2])],
            [-np.cos(pose_map_camera[2]), 0.0, np.sin(pose_map_camera[2])],
            [0.0,-1.0,0.0]]
        )
    trans_map_camera = np.array([pose_map_camera[0] ,pose_map_camera[1] ,0. ])
        
    # Compute R and t from map to apriltag
    T_map_apriltag = combine_transformations(
        rot_map_camera, trans_map_camera,
        rot_camera_apriltag, trans_camera_apriltag,
    )
    rot_map_apriltag, trans_map_apriltag = T_map_apriltag[:3, :3], T_map_apriltag[:3, 3]
    #rot_map_apriltag, trans_map_apriltag = projectApriltag(rot_map_apriltag, trans_map_apriltag)
    x,y,z,w = R.from_matrix(rot_map_apriltag).as_quat()
    return np.array([trans_map_apriltag[0], trans_map_apriltag[1],trans_map_apriltag[2],x, y, z, w])
