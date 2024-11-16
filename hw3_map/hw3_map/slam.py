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
from hw3_map.kalman import KalmanFilter
from hw3_map.utils import combine_transformations, findAprilTagMap, findInverse

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
        self.maximumValue = 0.02
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
        self.current_state = None
        self.pose_updated = False
        self.z = None
        self.poses_map_apriltag = []
        self.marker_ids = []
        self.obserbed_marker = []

    def set_current_state(self, current_state):
        self.current_state = current_state

    def april_pose_callback(self, msg):
        # Log the number of poses in the PoseArray
        self.pose_updated = False
        self.z = None
        self.poses_map_apriltag = []
        self.marker_ids = []
        #self.get_logger().info(f"Received PoseArray with {len(msg.poses)} poses")
        if len(msg.poses) < 1:
            return
        pose_ids = msg.header.frame_id.split(',')[:-1]
        zt = np.empty((0,1))
        for index, pose in enumerate(msg.poses):
            tag_id = pose_ids[index]
            pose_camera_apriltag = pose

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

            temp_z = np.append(trans_camera_apriltag[:3],1.).reshape(4,1)
            # New Marker
            if tag_id not in self.obserbed_marker:
                self.obserbed_marker.append(tag_id)
            else:
                zt = np.vstack((zt, temp_z))
                self.marker_ids.append(tag_id)

            temp_pose_map_apriltag = findAprilTagMap(quat_camera_apriltag, trans_camera_apriltag,self.current_state)
            pose_map_apriltag_t = np.append(temp_pose_map_apriltag[:3],1.).reshape(4,1)
            self.poses_map_apriltag.append({'id':tag_id,'pose':pose_map_apriltag_t})
        self.z = zt
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
    kalman_filter = KalmanFilter()
    waypoint = np.array([[0.8,0.0,0.0], 
                         #[0.8,0.8,np.pi/2],
                         #[0.0,0.8,np.pi],
                         #[0.0,0.0,0.0]
                         ])

    # init pid controller
    #0.034,0.005,0.005
    pid = PIDcontroller(0.034,0.005,0.005)
    current_state = kalman_filter.getPose()
    robot_state_estimator.set_current_state(current_state)
    update_value = np.array([0.0, 0.0, 0.0])
    for wp in waypoint:
        print("move to way point", wp)
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.12):
            # set wp as the target point
            pid.setTarget(wp)

            # Detect tag
            rclpy.spin_once(robot_state_estimator)
            found_state = robot_state_estimator.pose_updated

            # No Detection: Only Kalman Predict
            if not found_state:
                # Kalman Predict
                kalman_filter.kalmanPredict(coord(update_value, current_state))
                # set state
                current_state = kalman_filter.getPose()
                robot_state_estimator.set_current_state(current_state)
                # calculate the current twist
                update_value = pid.update(current_state)
                # publish the twist
                pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
                time.sleep(0.1)
                continue

            # With Detection: Kalman Predict + Kalman Update
            z, poses_map_apriltag, marker_ids =  robot_state_estimator.z, robot_state_estimator.poses_map_apriltag, robot_state_estimator.marker_ids
            # Kalman Predict
            kalman_filter.kalmanPredict(coord(update_value, current_state))
            print(kalman_filter.getPose())

            # Kalman Update
            if len(marker_ids) != 0: 
                kalman_filter.kalmanUpdate(z,marker_ids)
                print(kalman_filter.getPose())

            # Add new tag first
            kalman_filter.add_labels_to_state_and_covariance(poses_map_apriltag)

            # set state
            current_state = kalman_filter.getPose()
            robot_state_estimator.set_current_state(current_state)

            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            time.sleep(0.1)
        kalman_filter.displayMap()
        st, lt = kalman_filter.getMap()
        #print(st)
        #print(lt.diagonal())
    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    #robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
