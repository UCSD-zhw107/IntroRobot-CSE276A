"""
This file include code that control the robot motors
"""


from megapi import MegaPi
import math
import numpy as np
import time
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
Q0_ROTATION_GROUND = [(-33.5,-143), (33.5,139.4)]
Q1_ROTATION_GROUND = [(-32,-146), (32.4,140)]
Q2_ROTATION_GROUND = [(-32.7,-143), (34.6,139.4)]
Q3_ROTATION_GROUND = [(-31.6,-143.5), (33.55,144.4)]


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
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


    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))
        

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
    
    def inverseKinematic(self, input_w):
        """
        Transform the angular velocity of each wheel into desired robot linear
        velocity and angular velocity
        
        Param:
            w: a colum vector [w0,w1,w2,w3]

        Return
            vr: a colum vector [vx,vy,wz]
        """
        #Check w size
        w = []
        if input_w.shape == (4, 1):  # Column vector
            w = input_w.T.flatten()  # Transpose to row vector and flatten to 1D array
        elif input_w.shape == (1, 4):  # Row vector
            w = input_w.flatten()  # Flatten to 1D array
        else:
            raise AssertionError(f"Array shape must be (1, 4) or (4, 1)")
        #Get w0,w1,w2,w3
        w0 = w[self.pfl]
        w1 = w[self.pfr]
        w2 = w[self.pbl]
        w3 = w[self.pbr]
        #Get parameter
        r = self.r
        lx = self.lx
        ly = self.ly
        #Calculate vx, vy, wz
        vx = r/4 * (w0 + w1 + w2 + w3)
        vy = r/4 * (-w0 + w1 + w2 - w3)
        wz = r/(4 * (lx + ly)) * (-w0 + w1 - w2 + w3)
        #Calculate oriantation and speed
        vr = np.sqrt(vx**2 + vy**2)
        theta_r = math.atan2(vy,vx)
        return np.array([vx,vy,wz,vr,theta_r])

    def rotationToW(self,rotation):
        """
        Convert rotation / minutes to rad/s

        Param:
            rotation:rotation/minutes
        
        Return:
            rad/s
        """
        w1 = (rotation[0] * math.pi * 2) / 60
        w2 = (rotation[1] * math.pi * 2) / 60
        return w1, w2
    
    def linearApproximation(self, neg_par, pos_pair, deadzone,w):
        """
        Use Linear Approximation to map the angular velocity of wheel into
        control input of q or other way arround

        Param:
            neg_pair: pair of (q,w) for negative q
            pos_pair: pair of (q,w) for positive q
            deadzone: deadzone of q
            w: value of w
        
        Return:
            mapped q value
        """
        deadzone_neg = deadzone[0]-1
        deadzone_pos = deadzone[1]-1

        q_neg_high = neg_par[1][0]
        q_neg_low = neg_par[0][0]
        q_pos_high = pos_pair[1][0]
        q_pos_low = pos_pair[0][0]

        w_neg_high = neg_par[1][1]
        w_neg_low = neg_par[0][1]
        w_pos_high = pos_pair[1][1]
        w_pos_low = pos_pair[0][1]

        k_pos = (w_pos_high - w_pos_low) / ((q_pos_high - deadzone_pos) - (q_pos_low - deadzone_pos))
        k_neg = (w_neg_high - w_neg_low) / ((q_neg_high + deadzone_neg) - (q_neg_low + deadzone_neg))

        q = 0
        if(w > 0):
            q = (w / k_pos) + deadzone_pos
        elif(w < 0):
            q = (w / k_neg) + deadzone_neg

        if q in range(deadzone_neg, deadzone_pos):
            q = 0
        
        return int(round(q,0))
    
    def mapControlInput(self,input):
        """
        Map q into w for each wheel or w into q

        Param:
            input: input vector, either q or w
            task_type: [0,1] 0 for w to q, 1 for q to w

        Return
            converted q or w
        """
        # Range
        q_def = self.qdef

        # Input w
        w0 = input[self.pfl]
        w1 = input[self.pfr]
        w2 = input[self.pbl]
        w3 = input[self.pbr]

        q0 = 0
        q1 = 0
        q2 = 0
        q3 = 0

        ## Q0
        if w0 != 0:
            # Deadzone
            q0_brk_low = self.qbrk[self.pfl][0]
            q0_brk_high = self.qbrk[self.pfl][1]
            # Rotation
            q0_rot_low = self.q0rot[0]
            q0_w_low_min, q0_w_low_max = self.rotationToW(q0_rot_low)
            q0_w_high_min, q0_w_high_max = self.rotationToW(self.q0rot[1])
            # Build (q,w) pair
            q0_neg_pair = [(q0_brk_low, q0_w_low_min),(q_def[0], q0_w_low_max)]
            q0_pos_pair = [(q0_brk_high, q0_w_high_min),(q_def[1], q0_w_high_max)]
            q0 = self.linearApproximation(q0_neg_pair,q0_pos_pair,self.qbrk[self.pfl],w0)
        ## Q1
        if w1 != 0:
            # Deadzone
            q1_brk_low = self.qbrk[self.pfr][0]
            q1_brk_high = self.qbrk[self.pfr][1]
            # Rotation
            q1_w_low_min, q1_w_low_max = self.rotationToW(self.q1rot[0])
            q1_w_high_min, q1_w_high_max = self.rotationToW(self.q1rot[1])
            # Build (q,w) pair
            q1_neg_pair = [(q1_brk_low, q1_w_low_min),(q_def[0], q1_w_low_max)]
            q1_pos_pair = [(q1_brk_high, q1_w_high_min),(q_def[1], q1_w_high_max)]
            q1 = self.linearApproximation(q1_neg_pair,q1_pos_pair,self.qbrk[self.pfr],w1)
        
        ## Q2
        if w2 != 0:
            # Deadzone
            q2_brk_low = self.qbrk[self.pbl][0]
            q2_brk_high = self.qbrk[self.pbl][1]
            # Rotation
            q2_w_low_min, q2_w_low_max = self.rotationToW(self.q2rot[0])
            q2_w_high_min, q2_w_high_max = self.rotationToW(self.q2rot[1])
            # Build (q,w) pair
            q2_neg_pair = [(q2_brk_low, q2_w_low_min),(q_def[0], q2_w_low_max)]
            q2_pos_pair = [(q2_brk_high, q2_w_high_min),(q_def[1], q2_w_high_max)]
            q2 = self.linearApproximation(q2_neg_pair,q2_pos_pair,self.qbrk[self.pbl],w2)

        ## Q2
        if w3 != 0:
            # Deadzone
            q3_brk_low = self.qbrk[self.pbr][0]
            q3_brk_high = self.qbrk[self.pbr][1]
            # Rotation
            q3_w_low_min, q3_w_low_max = self.rotationToW(self.q3rot[0])
            q3_w_high_min, q3_w_high_max = self.rotationToW(self.q3rot[1])
            # Build (q,w) pair
            q3_neg_pair = [(q3_brk_low, q3_w_low_min),(q_def[0], q3_w_low_max)]
            q3_pos_pair = [(q3_brk_high, q3_w_high_min),(q_def[1], q3_w_high_max)]
            q3 = self.linearApproximation(q3_neg_pair,q3_pos_pair,self.qbrk[self.pbr],w3)

        return np.array([q0,q1,q2,q3])

        



            



    
    
            



    
    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-int(round(vfl,0)))
        self.bot.motorRun(self.mfr,int(round(vfr,0)))
        self.bot.motorRun(self.mbl,-int(round(vbl,0)))
        self.bot.motorRun(self.mbr,int(round(vbr,0)))

    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)

    def forwardMotion(self,vr, t):
        """
        Take in the linear velocity and angualr velocity and convert to q0,q1,q2,q3

        Param:
            input robot velocity
        
        Return:
            q
        """
        if self.verbose:
            print(f"CAR MOVE SPEED:{vr}")
        # Convert to w by forward kinematics
        w = self.forwardKinematic(vr)
        print(w)

        # Convert w to q
        q = self.mapControlInput(w)

        #Set Motor
        q0 = q[self.pfl]
        q1 = q[self.pfr]
        q2 = q[self.pbl]
        q3 = q[self.pbr]
        print(q)
        self.setFourMotors(q0,q1,q2,q3)
        time.sleep(t)

    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(speed, speed, speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()


if __name__ == "__main__":
    import time
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(1)
    mpi_ctrl.carStraight(-27)
    #mpi_ctrl.forwardMotion(np.array([[-0.3,0,-0.4]]))
    time.sleep(60)
    #mpi_ctrl.carSlide(30)
    #time.sleep(1)
    #mpi_ctrl.carRotate(30)
    #time.sleep(1)
    mpi_ctrl.carStop()
    mpi_ctrl.close()
