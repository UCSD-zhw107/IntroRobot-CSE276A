#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseArray,Pose2D
import numpy as np
import math
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from rclpy.time import Duration
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log
from hw5_system.utils import quaternion_from_euler, combine_transformations, quaternion_to_rotation_matrix, rotation_matrix_to_quaternion

class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        self.subscription = self.create_subscription(
            PoseArray,
            '/april_poses',
            self.april_pose_callback,
            10)
        self.apriltag_world_poses = {
            # pi
            'marker_5': (1.7, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5),
            'marker_2': (1.7, 0.8, 0.0, 0.5, -0.5, 0.5, -0.5),

            # 0
            'marker_3': (-0.9, 0.0, 0.0, -0.5, -0.5, 0.5, 0.5),
            'marker_4': (-0.9, 0.8, 0.0, -0.5, -0.5, 0.5, 0.5),

            # -pi/2 0.70710678, 0.0, 0.0, -0.70710678
            'marker_1': (0.36, 0.67, 0.0, 0.70710678, 0.0, 0.0, -0.70710678),
            'marker_9': (0.0, 1.7, 0.0, 0.70710678, 0.0, 0.0, -0.70710678),
            'marker_8': (0.64, 1.7, 0.0, 0.70710678, 0.0, 0.0, -0.70710678),
        }
        self.pose_publisher = self.create_publisher(Pose2D, '/robot_state', 10)

    def april_pose_callback(self, msg):
        if len(msg.poses) < 1:
            pose_msg = Pose2D()
            pose_msg.x = float('nan')
            pose_msg.y = float('nan')
            pose_msg.theta = float('nan')
            self.pose_publisher.publish(pose_msg)
            return
        pose_ids = msg.header.frame_id.split(',')[:-1]
        tag_id = pose_ids[0]
        pose_camera_apriltag = msg.poses[0]   
        trans_camera_apriltag = np.array([
            pose_camera_apriltag.position.x,
            pose_camera_apriltag.position.y,
            pose_camera_apriltag.position.z,
        ])
        quat_camera_apriltag = np.array([
            pose_camera_apriltag.orientation.x,
            pose_camera_apriltag.orientation.y,
            pose_camera_apriltag.orientation.z,
            pose_camera_apriltag.orientation.w,
        ])
        rot_camera_apriltag = quaternion_to_rotation_matrix(quat_camera_apriltag)

        # project to 2d
        trans_camera_apriltag_2d = trans_camera_apriltag
        trans_camera_apriltag_2d[1] = 0.0

        
        rot_camera_apriltag_2d = rot_camera_apriltag

        rot_apriltag_camera_2d = rot_camera_apriltag_2d.T
        trans_apriltag_camera_2d = -np.dot(rot_apriltag_camera_2d, trans_camera_apriltag_2d)
        
        # retrieve static transformation of apriltags wrt map
        rot_map_apriltag = quaternion_to_rotation_matrix(self.apriltag_world_poses[tag_id][3:])
        trans_map_apriltag = np.asarray(self.apriltag_world_poses[tag_id][:3])

        # apply transfomation
        T_map_camera = combine_transformations(
            rot_map_apriltag, trans_map_apriltag,
            rot_apriltag_camera_2d, trans_apriltag_camera_2d,
        )
        rot_map_camera, trans_map_camera = T_map_camera[:3, :3], T_map_camera[:3, 3]
        angle = math.atan2(rot_map_camera[1][2], rot_map_camera[0][2])
        pose_msg = Pose2D()
        pose_msg.x = trans_map_camera[0]
        pose_msg.y = trans_map_camera[1]
        pose_msg.theta = angle
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()