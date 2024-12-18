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
#KA = 0.8
#KP = 0.4
#KB = 0


"""KA = 0.55
KP = 1.4
KB = -0.2
"""

KA = 1.7
KP = 0.8
KB = -0.44

P_THRESHOLD = 0.3


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
        self.pthreshold = P_THRESHOLD


    def pathPlanning(self, dt):
        # Current pose
        current_pose = self.current_waypoint
        current_ind = self.waypoints_ind
        target = self.waypoints[current_ind]
        print(f'Target Pose: {target}')

        # Current robot position and orientation in world frame
        x_r, y_r, theta_r = current_pose[0], current_pose[1], current_pose[2]

        # Target position in world frame
        x_w, y_w, theta_w = target[0], target[1], target[2]
        dx = x_w - x_r
        dy = y_w - y_r
        p = np.sqrt(dx**2 + dy**2)
        a = math.atan2(math.sin(math.atan2(dy,dx)-theta_r),math.cos(math.atan2(dy,dx)-theta_r))
        b = np.arctan2(np.sin(theta_w - theta_r - a), np.cos(theta_w - theta_r - a))

        # Decide whether to move forward or backward
        if abs(a) == np.pi:
            a = 0
            b = 0
            p = -p

        v = self.kp * p
        gamma = self.ka * a if p > self.pthreshold else self.kb * b
        matrix_v_g = np.array([[v],[gamma]])
        matrix_rotate = np.array([[math.cos(current_pose[2]), 0], 
                                  [math.sin(current_pose[2]), 0], 
                                  [0,1]])
        res = np.dot(matrix_rotate, matrix_v_g)
        res = res.T.flatten()
        vx_w = res[0]
        vy_w = res[1]
        wz = res[2]
        print(f"vx: {v}, vy: {0}, wz: {wz}")

        # Update Pose
        x_r_update = x_r + vx_w * dt
        y_r_update = y_r + vy_w * dt
        theta_r_update = theta_r + wz * dt
        theta_r_update = (theta_r_update + np.pi) % (2 * np.pi) - np.pi
        self.current_waypoint = [x_r_update, y_r_update, theta_r_update]
        print(f'Current Post: {self.current_waypoint}')
        return [v, 0, wz, dt]

    def run(self):
        while True:
            # parse keyboard control
            key = get_key(self.settings, timeout=0.1)

            # interpret keyboard control as joy
            joy_msg, flag = self.key_to_joy(key)
            if flag is False:
                break

    
        self.stop()

    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if key == 'w':
            if self.waypoints_ind != len(self.waypoints):
                total_time = 3
                dt = 0.5
                accumulate_time = 0
                while(accumulate_time < total_time):
                    motion = self.pathPlanning(dt)
                    vx = motion[0]
                    vy = motion[1]
                    wz = motion[2]
                    t = motion[3]
                    joy_msg.axes[0] = vx
                    joy_msg.axes[1] = vy
                    joy_msg.axes[2] = wz
                    joy_msg.axes[3] = t
                    joy_msg.axes[4] = 1
                    accumulate_time += dt
                    self.publisher_.publish(joy_msg)
                    time.sleep(t + 0.5)
                self.waypoints_ind += 1
                print("MOVE DONE")
                joy_msg.axes[4] = -1.0
                self.publisher_.publish(joy_msg)
            else:
                joy_msg.axes[4] = 2
        elif key == 's':
            joy_msg.axes[4] = -1.0
            self.publisher_.publish(joy_msg)
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