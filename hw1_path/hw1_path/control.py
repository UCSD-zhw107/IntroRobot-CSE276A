#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import math
from typing import List
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseArray,Pose2D
from threading import Lock

class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd, waypoints: List[np.ndarray]):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.03
        # MPI Twist
        self.publisher = self.create_publisher(Twist, '/twist', 10)
        # Pose Esitmation
        self.state_subscription = self.create_subscription(
            Pose2D,
            '/robot_state',
            self.state_callback,
            10)
        # Plan
        self.plan_subscription = self.create_subscription(
            Pose2D,
            '/planned_target',
            self.plan_callback,
            10)
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.lock = Lock()
        self.waypoints = waypoints
        
    def genTwistMsg(self,desired_twist):
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


    def coord(self,twist, current_state):
        J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                    [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                    [0.0,0.0,1.0]])
        return np.dot(J, twist)

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
    
    def state_callback(self, msg):
        '''
        Robot state estimation
        '''
        # No Observation
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.theta):
            return
        try:
            self.lock.acquire()
            self.current_state = np.array([msg.x, msg.y, msg.theta])
        finally:
            self.lock.release()

    def plan_callback(self,msg):
        '''
        Robot Plan
        '''
        # Finish
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.theta):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            raise SystemExit
        self.setTarget([msg.x, msg.y, msg.theta])
    
    def timer_callback(self):
        if len(self.waypoints) == 0:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            raise SystemExit
        try:
            self.lock.acquire()
            current_state_copy = self.current_state.copy()
        finally:
            self.lock.release()
        if self.target is None or np.linalg.norm(self.getError(current_state_copy, self.target)) < 0.2:
            self.setTarget(self.waypoints.pop(0))
        else:
            print(self.target)
            update_value = self.update(current_state_copy)
            self.publisher.publish(self.genTwistMsg(self.coord(update_value, current_state_copy)))
            try:
                self.lock.acquire()
                self.current_state += update_value
            finally:
                self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    waypoints_square = [
        np.array([1.0, 0.0, 0.0]),
        #np.array([1.0, 0.0, np.pi/2]),
        np.array([1.0, 1.0, np.pi/2]),
        #np.array([1.0, 1.0, np.pi]),
        np.array([0.0, 1.0, np.pi]),
        #np.array([0.0, 1.0, -np.pi/2]),
        np.array([0.0, 0.0, -np.pi/2]),
        np.array([0.0, 0.0, 0.0])
    ]

    node = PIDcontroller(0.1,0.01,0.000, waypoints=waypoints_square)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected. Shutting down...')
    except SystemExit:
        node.get_logger().info('Finished Job. Shutting down...')
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        node.publisher.publish(twist)
        node.destroy_node()
        rclpy.shutdown()