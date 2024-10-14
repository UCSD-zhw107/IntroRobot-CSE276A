"""
This file include code that control the robot motors
"""


from megapi import MegaPi
import math
import numpy as np
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
        self.alpha = ALPHA
        self.beta = BETA
        self.gamma = GAMMA
        self.lx = LX
        self.ly = LY
        self.r = R


    
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
        vx = vr[0]
        vy = vr[1]
        wz = vr[2]
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
        w0 = w[0]
        w1 = w[1]
        w2 = w[2]
        w3 = w[3]
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
    
    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,-vbl)
        self.bot.motorRun(self.mbr,vbr)

    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)
    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


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
    mpi_ctrl.carStraight(30)
    time.sleep(4)
    #mpi_ctrl.carSlide(30)
    #time.sleep(1)
    #mpi_ctrl.carRotate(30)
    #time.sleep(1)
    mpi_ctrl.carStop()
    mpi_ctrl.close()
