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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from hw1_path.key_parser import get_key, save_terminal_settings, restore_terminal_settings
import numpy as np

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
            joy_msg.axes[1] = 1.0
        elif key == 's':
            joy_msg.axes[1] = -1.0
        elif key == 'a':
            joy_msg.axes[0] = -1.0
        elif key == 'd':
            joy_msg.axes[0] = 1.0
        elif key == 'q':
            joy_msg.axes[2] = 1.0
        elif key == 'e':
            joy_msg.axes[2] = -1.0
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