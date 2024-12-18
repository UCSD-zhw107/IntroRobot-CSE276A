#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseArray
import numpy as np
import math
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from rclpy.time import Duration
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log
from hw5_system.utils import quaternion_from_euler, combine_transformations, quaternion_to_rotation_matrix, rotation_matrix_to_quaternion

class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.04
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)

    def setTarget(self, target):
        """
        Set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array(target)

    def getError(self, currentState, targetState):
        """
        Return the difference between two states.
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        Set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        Calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)
        P = self.Kp * e
        self.I += self.Ki * e * self.timestep
        D = self.Kd * (e - self.lastError)
        result = P + self.I + D
        self.lastError = e

        # Scale down the twist if its norm is more than the maximum value
        resultNorm = np.linalg.norm(result)
        if resultNorm > self.maximumValue:
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result


class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        self.subscription = self.create_subscription(
            PoseArray,
            '/april_poses',
            self.april_pose_callback,
            10)
        self.subscription
        self.br = TransformBroadcaster(self)
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.pose_updated = True

        # if you are not using tf2 service, define your aptiltag poses (wrt map frame) here, z value is omitted to ensure 2D transformation
        self.apriltag_world_poses = {
            # 0
            'marker_5': (1.7, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5),
            'marker_2': (1.7, 0.8, 0.0, 0.5, -0.5, 0.5, -0.5),

            # -pi
            'marker_3': (-0.9, 0.0, 0.0, -0.5, -0.5, 0.5, 0.5),
            'marker_4': (-0.9, 0.8, 0.0, -0.5, -0.5, 0.5, 0.5),
            # -pi/2 -0.5, 0.5, 0.5, 0.5
            'marker_1': (0.36, 0.67, 0.0, -0.5, 0.5, 0.5, 0.5),
            'marker_9': (0.0, 1.7, 0.0, 0.70710678, 0.0, 0.0, -0.70710678),
            'marker_8': (0.64, 1.7, 0.0, 0.70710678, 0.0, 0.0, -0.70710678),
        }


    def april_pose_callback(self, msg):
        # Log the number of poses in the PoseArray
        self.pose_updated = False
        #self.get_logger().info(f"Received PoseArray with {len(msg.poses)} poses")
        if len(msg.poses) < 1:
            return
        pose_ids = msg.header.frame_id.split(',')[:-1]
        
        # we will only use one landmark at a time in homework 2. in homework 3, all landmarks should be considered.
        
        tag_id = pose_ids[0]
        pose_camera_apriltag = msg.poses[0]   # syntax: pose_ReferenceFrame_TargetFrame
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

        self.current_state = np.array([trans_map_camera[0], trans_map_camera[1], angle])
        self.pose_updated = True


def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg


def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)


def main(args=None):
    rclpy.init(args=args)
    robot_state_estimator = RobotStateEstimator()

    start_point = [0, 0]
    goal_point = [0.9, 1.37]
    obstacle = np.array([[0.19, 0.67],
                         [0.53, 0.67],
                         [0.53, 0.93],
                         [0.19, 0.93]])
    boundary = np.array([
        [1.7, -0.74],
        [-0.9, -0.74],
        [-0.9, 1.7],
        [1.7, 1.7]
    ])
    #v = voronoi_planner(obstacle, boundary,start_point, goal_point)
    #v = visibility_planner(obstacle, boundary,start_point, goal_point, radius=0.13)
    #waypoint = v.plan_path()[1:]
    '''waypoint = np.array([
        [1, 0 ,0],
        [1, 0.2, np.pi/2],
        [0, 0.2, np.pi],
        [0, 0.4, np.pi/2],
        [1, 0.4, 0]
    ])'''
    
    waypoint = np.array([
        [1.2, -0.4, 0],
        [1.2, -0.2, np.pi/2],
        [-0.4, -0.2, np.pi],
        [-0.4, 0.0, np.pi/2],
        [1.2, 0 ,0],
        [1.2, 0.2, np.pi/2],
        [-0.4, 0.2, np.pi],
        [-0.4, 0.4, np.pi/2],
        [1.2, 0.4, 0],
        [1.2, 0.6, np.pi/2],
        [-0.4, 0.6, np.pi],
        [-0.4, 0.8, np.pi/2],
        [1.2, 0.8, 0],
        [1.2, 1.0, np.pi/2],
        [-0.4, 1.0, np.pi],
        [-0.4, 1.2, np.pi/2],
        [1.2, 1.2, 0],
        [1.2, 1.4, np.pi/2],
    ])
    
    
    print(f'WayPoint: {waypoint}')

    # init pid controller
    #0.034,0.005,0.005
    pid = PIDcontroller(0.04,0.005,0.005)
    current_state = robot_state_estimator.current_state
    trajectory = []

    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        rclpy.spin_once(robot_state_estimator)
        found_state, estimated_state = robot_state_estimator.pose_updated, robot_state_estimator.current_state
        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state
            print(f'Current: {estimated_state}')
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.2): # check the error between current state and current way point
            #print(f'Error: {np.linalg.norm(pid.getError(current_state, wp))}')
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            rclpy.spin_once(robot_state_estimator)
            found_state, estimated_state = robot_state_estimator.pose_updated, robot_state_estimator.current_state
            if found_state: # if the tag is detected, we can use it to update current state.
                current_state = estimated_state
                trajectory.append(current_state)
                print(f'Current: {estimated_state}')
        np.save('trajct.npy', np.asanyarray(trajectory))
        print(np.asanyarray(trajectory))
    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    #robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
