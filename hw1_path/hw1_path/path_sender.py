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
KB = -0.4


# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left



#Parameters of robot based on measurement
#Index
PFL = 0 #index for motor front left
PFR = 1 #index for motor front right
PBL = 2 #index for motor back left
PBR = 3 #index for motor back right

PVX = 0 #index for vx
PVY = 1 #index for vy
PWZ = 2 #index for wz

#Alpha: angle between robot center and wheel center
ALPHA = [math.pi/4 , -(math.pi/4), (3*math.pi)/4, -(3*math.pi)/4]

#Beta: angle between the y-axis of each wheel and the x-axis of robot frame
BETA = [math.pi/2, -(math.pi)/2, math.pi/2, -(math.pi)/2]

#Gamma: angle between the direction of passive wheel and x-axis of wheel
GAMMA = [-(math.pi)/4, math.pi/4, math.pi/4, -(math.pi)/4]

#lx: length between the wheel and x-axis of the robot frame
LX = 6.8/100

#ly: length between the wheel and y-axis of the robot frame
LY = 5.425/100

#R: the radius of wheels
R = 3/100


##Below need to be tuned
#Q_SCALE: scale factor for q
Q_SCALE = 1

#Q_DEFAULT: default range of q [-50,50]
Q_DEFAULT = (-100*Q_SCALE,100*Q_SCALE)

#Q_BRAEK: the range of q for each wheel where w=0
Q_BREAK = [(-27*Q_SCALE,27*Q_SCALE), (-27*Q_SCALE,27*Q_SCALE), (-27*Q_SCALE,27*Q_SCALE), (-27*Q_SCALE,27*Q_SCALE)]
#Q_BREAK_GROUND = [(-49*Q_SCALE,47*Q_SCALE), (-47*Q_SCALE,47*Q_SCALE), (-49*Q_SCALE,49*Q_SCALE), (-49*Q_SCALE,46*Q_SCALE)]
#Q_BREAK = [(-27*Q_SCALE,27*Q_SCALE), (-27*Q_SCALE,27*Q_SCALE), (-26*Q_SCALE,26*Q_SCALE), (-26*Q_SCALE,26*Q_SCALE)]

# W measured on hand
Q0_ROTATION = [(-35,-153), (34.5,152.3)]
Q1_ROTATION = [(-32.9,-142), (32.7,142)]
Q2_ROTATION = [(-34.4,-151.5), (34.7,152)]
Q3_ROTATION = [(-34.2,-154), (34.5,154.5)]


# W measured on ground
Q0_ROTATION_GROUND_POS = [(27,33.5),(55,71.8),(75,100), (100,139.4)]
Q0_ROTATION_GROUND_NEG = [(-27,-33.5),(-55,-72.4),(-75,-104), (-100,-143)]

Q1_ROTATION_GROUND_POS = [(27,32.4),(55,71.7),(75,100.7), (100,140)]
Q1_ROTATION_GROUND_NEG = [(-27,-32), (-55,-71.5),(-75,-101.2), (-100,-146)]

Q2_ROTATION_GROUND_POS = [(27,34.6),(55,73),(75,105.3), (100,139.4)]
Q2_ROTATION_GROUND_NEG = [(-27,-32.7), (-55,-71.2),(-75,-101.4),(-100,-143)]

Q3_ROTATION_GROUND_POS = [(27,33.55),(55,72.7),(75,104),(100,144.4)]
Q3_ROTATION_GROUND_NEG = [(-27,-31.6), (-55,-71.6),(-75,-101),(-100,-143.5)]

Q0_ROTATION_GROUND = [(-33.5,-143), (33.5,139.4)]
Q1_ROTATION_GROUND = [(-32,-146), (32.4,140)]
Q2_ROTATION_GROUND = [(-32.7,-143), (34.6,139.4)]
Q3_ROTATION_GROUND = [(-31.6,-143.5), (33.55,144.4)]


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
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left
        self.pfl = PFL	
        self.pfr = PFR
        self.pbl = PBL
        self.pbr = PBR
        self.pvx = PVX
        self.pvy = PVY
        self.pwz = PWZ
        self.alpha = ALPHA
        self.beta = BETA
        self.gamma = GAMMA
        self.lx = LX
        self.ly = LY
        self.r = R
        self.qdef = Q_DEFAULT
        self.qbrk = Q_BREAK
        self.qscale = Q_SCALE
        self.q0rot = Q0_ROTATION_GROUND
        self.q1rot = Q1_ROTATION_GROUND
        self.q2rot = Q2_ROTATION_GROUND
        self.q3rot = Q3_ROTATION_GROUND



    def forwardKinematic(self, input_vr):
        """
        Transform the desired robot linear velocity and angular velocity
        into angular velocity of each wheel

        Param:
            vr: a colum vector [vx, vy, wr]

        Return
            w: a colum vector [w0,w1,w2,w3]
        """
        #Check vr size
        vr = []
        if input_vr.shape == (3, 1):  # Column vector
            vr = input_vr.T.flatten()  # Transpose to row vector and flatten to 1D array
        elif input_vr.shape == (1, 3):  # Row vector
            vr = input_vr.flatten()  # Flatten to 1D array
        else:
            raise AssertionError(f"Array shape must be (1, 3) or (3, 1)")
        #Get vx,vy,wz
        vx = vr[self.pvx]
        vy = vr[self.pvy]
        wz = vr[self.pwz]
        #Get parameter
        r = self.r
        lx = self.lx
        ly = self.ly
        #Calcualte w0,w1,w2,w3
        w0 = 1/r * (vx - vy - (lx+ ly)*wz)
        w1 = 1/r * (vx + vy + (lx+ ly)*wz)
        w2 = 1/r * (vx + vy - (lx + ly)*wz)
        w3 = 1/r * (vx - vy + (lx + ly)*wz)
        return np.array([w0,w1,w2,w3])


    def get_wheel_regressions(self):

        def rotate(r):
            return (r * math.pi * 2) / 60
        
        # Data for q > 26
        q_pos = np.array([27, 55, 75, 100])
        w0_pos = np.array([33.5, 71.8, 100, 139.4])
        w1_pos = np.array([32.4, 71.7, 100.7, 140])
        w2_pos = np.array([34.6, 73, 105.3, 139.4])
        w3_pos = np.array([33.55, 72.7, 104, 144.4])

        # Data for q < -26
        q_neg = np.array([-27, -55, -75, -100])
        w0_neg = np.array([-33.5, -72.4, -104, -143])
        w1_neg = np.array([-32, -71.5, -101.2, -146])
        w2_neg = np.array([-32.7, -71.2, -101.4, -143])
        w3_neg = np.array([-31.6, -71.6, -101, -143.5])

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

        return wheel_regressions
            

    def predict_motor(self, input_w):
        regressions = self.get_wheel_regressions()
        convert_q = []

        for index, w in enumerate(input_w):
            wheel_key = f'q{index}'
            q = 0
            if w == 0:  # Handle dead zone where w = 0
                q = 0
            elif w > 0:  # For w corresponding to q > 26
                slope, intercept = regressions[wheel_key]["pos"]
                q = (w - intercept) / slope
                if q <= 26:  # If q falls into the dead zone, return 0
                    q = 0
            elif w < 0:  # For w corresponding to q < -26
                slope, intercept = regressions[wheel_key]["neg"]
                q = (w - intercept) / slope
                if q >= -26:  # If q falls into the dead zone, return 0
                    q = 0
            convert_q.append(q)
        return convert_q
    

    """def pathPlanning(self, dt):
        # Current pose
        current_pose = self.current_waypoint
        current_ind = self.waypoints_ind
        target = self.waypoints[current_ind]
        print(f'Target Pose: {target}')

        # Current robot position and orientation in world frame
        x_r, y_r, theta_r = current_pose[0], current_pose[1], current_pose[2]

        # Target position in world frame
        x_w, y_w, theta_w = target[0], target[1], target[2]

        # Convert the target from world frame to robot frame
        dx = (x_w - x_r) * np.cos(theta_r) + (y_w - y_r) * np.sin(theta_r)
        dy = -(x_w - x_r) * np.sin(theta_r) + (y_w - y_r) * np.cos(theta_r)

        p = np.sqrt(dx**2 + dy**2)
        #p_theta = math.atan2(dy,dx)
        #p_theta = math.atan2(dy,dx)
        #a = math.atan2(math.sin(p_theta-0),math.cos(p_theta-0))
        a = math.atan2(dy,dx)
        a = (a + np.pi) % (2 * np.pi) - np.pi
        b = np.arctan2(np.sin(theta_w - theta_r), np.cos(theta_w - theta_r))

        # Decide whether to move forward or backward
        if abs(a) == np.pi:
            a = 0
            b = 0
            p = -p

        v = self.kp * p
        gamma = self.ka * a + self.kb * b
        matrix_v_g = np.array([[v],[gamma]])
        matrix_rotate = np.array([[math.cos(a), 0], 
                                  [math.sin(a), 0], 
                                  [0,1]])
        res = np.dot(matrix_rotate, matrix_v_g)
        res = res.T.flatten()
        vx = res[0]
        vy = res[1]
        wz = res[2]
        print(f"vx: {vx}, vy: {vy}, wz: {wz}")
        
        vx_w = vx * math.cos(theta_r) - vy * math.sin(theta_r)
        vy_w = vx* math.sin(theta_r) + vy * math.cos(theta_r)
        x_r_update = x_r + vx_w * dt
        y_r_update = y_r + vy_w * dt
        theta_r_update = theta_r + wz * dt
        theta_r_update = (theta_r_update + np.pi) % (2 * np.pi) - np.pi
        vr = np.array([[vx, vy, wz]])
        w = self.forwardKinematic(vr)
        q = self.predict_motor(w)
        q0 = q[self.pfl]
        q1 = q[self.pfr]
        q2 = q[self.pbl]
        q3 = q[self.pbr]
        print(f'q: {q}')
        if abs(q0) > 28 and abs(q1) > 28 and abs(q2) > 28 and  abs(q3) > 28:
            self.current_waypoint = [x_r_update, y_r_update, theta_r_update]
        print(f'Current Post: {self.current_waypoint}')
        return [vx, vy, wz, dt]"""


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
        #p_theta = math.atan2(dy,dx)
        #p_theta = math.atan2(dy,dx)
        a = math.atan2(math.sin(math.atan2(dy,dx)-theta_r),math.cos(math.atan2(dy,dx)-theta_r))
        b = np.arctan2(np.sin(theta_w - theta_r - a), np.cos(theta_w - theta_r - a))

        # Decide whether to move forward or backward
        if abs(a) == np.pi:
            a = 0
            b = 0
            p = -p

        v = self.kp * p
        gamma = self.ka * a + self.kb * b
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
                #self.current_waypoint[0] = self.waypoints[self.waypoints_ind][0]
                #self.current_waypoint[1] = self.waypoints[self.waypoints_ind][1]
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