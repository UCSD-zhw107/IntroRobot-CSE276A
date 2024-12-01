import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseArray,Pose2D
import numpy as np
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log

class RobotPlanner(Node):
    def __init__(self):
        super().__init__('robot_planner')
        self.radius = 0.5
        self.boundary = np.array([-0.9, 1.7])
        self.subscription = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.pose_callback,
            10)
        self.task_publisher = self.create_publisher(Pose2D, '/planned_target', 10)
        self.turn_increment = 0.2
        self.just_turned = False
    
    def is_near_boundary(self, val):
        lower_bound, upper_bound = self.boundary
        if abs(val - lower_bound - self.radius) < 0.2:
            return True
        if abs(val - upper_bound + self.radius) < 0.2:
            return True
        return False
    
    def pose_callback(self,msg):
        robot_x = msg.x
        robot_y = msg.y
        robot_theta = msg.theta
        lower_bound,upper_bound = self.boundary

        # Reach Target, Stop
        if abs(robot_y - upper_bound) < self.radius:
            target_msg = Pose2D()
            target_msg.x = float('nan')
            target_msg.y = float('nan')
            target_msg.theta = float('nan')
            self.task_publisher.publish(target_msg)
            return

        # Turn Around
        if self.is_near_boundary(robot_x) and self.just_turned == False:
            self.just_turned = True
            target_msg = Pose2D()
            target_msg.x = robot_x
            target_msg.y = robot_y + self.turn_increment
            target_msg.theta = float(np.pi)/2
            self.task_publisher.publish(target_msg)
        else:
            # Move forward
            self.just_turned = False
            target_msg = Pose2D()
            target_msg.x = upper_bound-self.radius if robot_x < 0.5 else lower_bound + self.radius
            target_msg.y = robot_y
            target_msg.theta = 0.0 if robot_x < 0.5 else float(np.pi)
            self.task_publisher.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()