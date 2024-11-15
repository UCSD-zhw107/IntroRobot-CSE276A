#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import math
import rclpy # replaces rospy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from hw3_map.mpi_control import MegaPiController
import numpy as np


class MegaPiControllerNode(Node):
    def __init__(self, verbose=True, debug=False):
        super().__init__('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.r = 0.03 # radius of the wheel
        self.lx = 0.05425 # half of the distance between front wheel and back wheel
        self.ly = 0.068 # half of the distance between left wheel and right wheel
        self.calibration = 24
        #18.0 with regression
        self.subscription = self.create_subscription(Twist, '/twist', self.twist_callback, 10)
        self.regression = {}

    def get_wheel_regressions(self):

        def rotate(r):
            return (r * math.pi * 2) / 10
        
        # Data for q > 26
        q_pos = np.array([26, 55, 75, 100])
        w0_pos = np.array([3.6, 11.2, 16.2, 22.6])
        w1_pos = np.array([3.6, 11.0, 16.1, 22.5])
        w2_pos = np.array([3.7, 11.25, 16.5, 23])
        w3_pos = np.array([3.6, 11.14, 16.3, 22.5])

        # Data for q < -26
        q_neg = np.array([-26, -55, -75, -100])
        w0_neg = np.array([-3.5, -9.7, -16, -22.3])
        w1_neg = np.array([-3.5, -9.5, -15.8, -22.3])
        w2_neg = np.array([-3.5, -9.7, -17, -22.5])
        w3_neg = np.array([-3.5, -9.7, -16, -22.3])

        w0_pos_rad = np.array([rotate(w) for w in w0_pos])
        w1_pos_rad = np.array([rotate(w) for w in w1_pos])
        w2_pos_rad = np.array([rotate(w) for w in w2_pos])
        w3_pos_rad = np.array([rotate(w) for w in w3_pos])

        w0_neg_rad = np.array([rotate(w) for w in w0_neg])
        w1_neg_rad = np.array([rotate(w) for w in w1_neg])
        w2_neg_rad = np.array([rotate(w) for w in w2_neg])
        w3_neg_rad = np.array([rotate(w) for w in w3_neg])
        # Perform linear regression for each set of data
        def perform_regression(q_values, w_values):
            # Calculate the slope and intercept using least squares formula
            n = len(q_values)
            q_mean = np.mean(q_values)
            w_mean = np.mean(w_values)
            
            numerator = np.sum((q_values - q_mean) * (w_values - w_mean))
            denominator = np.sum((q_values - q_mean) ** 2)
            slope = numerator / denominator
            intercept = w_mean - slope * q_mean
            return slope, intercept

        # Generate regression parameters for each wheel and each range
        wheel_regressions = {
            "q0": {"pos": perform_regression(q_pos, w0_pos_rad), "neg": perform_regression(q_neg, w0_neg_rad)},
            "q1": {"pos": perform_regression(q_pos, w1_pos_rad), "neg": perform_regression(q_neg, w1_neg_rad)},
            "q2": {"pos": perform_regression(q_pos, w2_pos_rad), "neg": perform_regression(q_neg, w2_neg_rad)},
            "q3": {"pos": perform_regression(q_pos, w3_pos_rad), "neg": perform_regression(q_neg, w3_neg_rad)},
        }

        self.regression = wheel_regressions
    

    def predict_motor(self, input_w):
        regressions = self.regression
        convert_q = []

        for index, w in enumerate(input_w):
            wheel_key = f'q{index}'
            q = 0
            if w == 0:  # Handle dead zone where w = 0
                q = 0
            elif w > 0:  # For w corresponding to q > 26
                slope, intercept = regressions[wheel_key]["pos"]
                q = (w - intercept) / slope
                if q <= 25:  # If q falls into the dead zone, return 0
                    q = 0
            elif w < 0:  # For w corresponding to q < -26
                slope, intercept = regressions[wheel_key]["neg"]
                q = (w - intercept) / slope
                if q >= -25:  # If q falls into the dead zone, return 0
                    q = 0
            convert_q.append(q)
        return convert_q

    
    def twist_callback(self, twist_cmd):
        desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)
        w = np.array([result[0][0],result[1][0],result[2][0],result[3][0]])
        q = self.predict_motor(w)
        # send command to each wheel
        self.mpi_ctrl.setFourMotors(q[0],q[1],q[2],q[3])

def main(args=None):
    rclpy.init(args=args)
    mpi_ctrl_node = MegaPiControllerNode()
    mpi_ctrl_node.get_wheel_regressions()
    #rospy.init_node('megapi_controller')
    #rospy.Subscriber('/twist', Twist, mpi_ctrl_node.twist_callback, queue_size=1) 

    
    rclpy.spin(mpi_ctrl_node) # Spin for until shutdown

    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()