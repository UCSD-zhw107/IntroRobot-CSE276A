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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from hw1_path.key_parser import get_key, save_terminal_settings, restore_terminal_settings
import numpy as np

# Gain
KA = 1
KP = 1
KB = 1

class KeyJoyNode(Node):
    def __init__(self):
        super().__init__('key_joy_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 1)
        self.settings = save_terminal_settings()
        self.waypoints = np.array([
            [-1,0,0],
            [-1,1,1.57]
            [-2,1,0],
            [-2,2,-1.57]
            [0,0,0]   
        ])
        self.current_waypoint = [0,0,0]
        self.waypoints_ind = 0
        self.ka = KA
        self.kp = KP
        self.kb = KB

    def pathPlanning(self):
        # Current pose
        current_pose = self.current_waypoint
        current_ind = self.waypoints_ind
        target = self.waypoints[current_ind]

        # Calcualte p, a, b
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        p = np.sqrt(dx**2 + dy**2)
        p_theta = math.atan2(dy,dx)
        a = math.atan2(math.sin(p_theta-current_pose[2]),math.cos(p_theta-current_pose[2]))
        b = math.atan2(math.sin(target[2]-current_pose[2]),math.cos(target[2]-current_pose[2]))

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
        v_total = np.sqrt(vx**2 + vy **2)
        t_linear = p / v_total
        t_angular = b / wz
        t = max(t_linear, t_angular)
        return [vx,vy,wz,t]

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
                self.current_waypoint = self.waypoints[self.waypoints_ind]
                vx = motion[0]
                vy = motion[1]
                wz = motion[2]
                t = motion[3]
                joy_msg.axes[0] = vx
                joy_msg.axes[1] = vy
                joy_msg.axes[2] = wz
                joy_msg.axes[3] = t
                joy_msg.axes[4] = 1
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