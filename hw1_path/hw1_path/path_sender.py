#!/usr/bin/env python3
"""
Copyright 2023 UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from hw1_path.key_parser import get_key, save_terminal_settings, restore_terminal_settings
import numpy as np

# Gain
KA = 1.1
KP = 1.0
KB = -1.0

class KeyJoyNode(Node):
    def __init__(self):
        super().__init__('key_joy_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 1)
        self.settings = save_terminal_settings()
        self.waypoints = [
            [-1,0,0],
            [-1,1,1.57],
            [-2,1,0],
            [-2,2,-1.57],
            [0,0,0]   
        ]
        self.current_waypoint = [0,0,0]
        self.waypoints_ind = 0
        self.ka = KA
        self.kp = KP
        self.kb = KB

    def pathPlanning(self):
        # Current pose
        current_pose = self.current_waypoint
        print(f'Current Post: {current_pose}')
        current_ind = self.waypoints_ind
        target = self.waypoints[current_ind]
        print(f'Target Pose: {target}')

        # Current robot position and orientation in world frame
        x_r, y_r, theta_r = current_pose[0], current_pose[1], current_pose[2]

        # Target position in world frame
        x_w, y_w, theta_w = target[0], target[1], target[2]

        # Step 1: Convert the target from world frame to robot frame
        dx = (x_w - x_r) * np.cos(theta_r) + (y_w - y_r) * np.sin(theta_r)
        dy = -(x_w - x_r) * np.sin(theta_r) + (y_w - y_r) * np.cos(theta_r)

        p = np.sqrt(dx**2 + dy**2)
        p_theta = math.atan2(dy,dx)
        a = math.atan2(math.sin(p_theta-0),math.cos(p_theta-0))
        b = np.arctan2(np.sin(theta_w - theta_r - a), np.cos(theta_w - theta_r - a))

        v = self.kp * p
        gamma = self.ka * a + self.kb * b
        matrix_v_g = np.array([[v],[gamma]])
        matrix_rotate = np.array([[math.cos(current_pose[2]), 0], 
                                  [math.sin(current_pose[2]), 0], 
                                  [0,1]])
        res = np.dot(matrix_rotate, matrix_v_g)
        res = res.T.flatten()
        vx = res[0]
        vy = res[1]
        wz = res[2]
        print(f"vx: {vx}, vy: {vy}, wz: {wz}")

        v_total = np.sqrt(vx**2 + vy **2)

        if v_total == 0:
            print("v_total is 0, cannot divide by 0")
            t_linear = 0
        else:
            t_linear = abs(p) / abs(v_total)

        print(f"v_total: {v_total}, t_linear: {t_linear}")

        if wz == 0:
            print("wz is 0, cannot divide by 0")
            t_angular = 0
        else:
            t_angular = abs(b) / abs(wz)

        print(f"t_angular: {t_angular}")

        t = max(t_linear, t_angular)
        print(f"Time to reach the waypoint: {t}")

        x_r += vx * t
        y_r += vy * t
        theta_r += wz * t
        theta_r = (theta_r + np.pi) % (2 * np.pi) - np.pi
        self.current_waypoint = [x_r, y_r, theta_r]


        return [vx, vy, wz, t]

    def run(self):
        while True:
            # parse keyboard control
            key = get_key(self.settings, timeout=0.1)

            # interpret keyboard control as joy
            joy_msg, flag = self.key_to_joy(key)
            if flag is False:
                break

            # publish joy
            self.publisher_.publish(joy_msg)
    
        self.stop()

    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if key == 'w':
            if self.waypoints_ind != len(self.waypoints) - 1:
                motion = self.pathPlanning()
                self.waypoints_ind += 1
                vx = motion[0]
                vy = motion[1]
                wz = motion[2]
                t = motion[3]
                joy_msg.axes[0] = vx
                joy_msg.axes[1] = vy
                joy_msg.axes[2] = wz
                joy_msg.axes[3] = t
                joy_msg.axes[4] = 1
                time.sleep(t)
                print("MOVE DONE")
            else:
                joy_msg.axes[4] = 2
        elif key == 's':
            joy_msg.axes[4] = -1.0
        elif (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
            flag = False
        return joy_msg, flag
    

    def stop(self):
        restore_terminal_settings(self.settings)


def main(args=None):
    rclpy.init(args=args)
    key_joy_node = KeyJoyNode()
    key_joy_node.run()
    key_joy_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()