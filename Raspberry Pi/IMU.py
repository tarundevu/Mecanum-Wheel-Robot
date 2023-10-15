import time
import math

class Mpu():
    def __init__(self):

        self.Roll  = 0.0
        self.Pitch  = 0.0
        self.Yaw  = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.heading = 0.0
        self.cur_w = 0.0
        self.inital_yaw = 0.0
    
    def setInitialYaw(self,yaw):
        self.inital_yaw = yaw

    def getOdometry(self,data):
        pose = [0.0,0.0,0.0]
        yaw = data[2]
        ### heading, W ###
        change = yaw - self.inital_yaw

        if change > 180:
            change -= 360
        elif change < -180:
            change += 360

        self.cur_w += change

        self.heading = round(yaw,2)
        self.cur_w = round(math.radians(self.cur_w),2)
        ### X and Y ###



        
        self.inital_yaw = yaw

        return float(self.heading), float(self.cur_w)
