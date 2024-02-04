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
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_w = 0.0
        self.initial_yaw = 0.0
        self.prev_yaw = 0.0
        self.prev_time = 0.0
        self.cal = 10
    
    def setInitialYaw(self,yaw):
        self.initial_yaw = yaw
        
    def getOdometry(self,data,cur_time):
        yaw = data[2]

        ### heading, W ###
        calibrated_yaw = yaw - self.initial_yaw

        if calibrated_yaw > 180:
            calibrated_yaw -= 360
        elif calibrated_yaw < -180:
            calibrated_yaw += 360

        change =  calibrated_yaw - self.prev_yaw

        if change > 180:
            change -= 360
        elif change < -180:
            change += 360
       
        self.cur_w += change

        self.heading = round(calibrated_yaw,2)
        
        self.prev_yaw = calibrated_yaw
        self.prev_time = cur_time

        return round(math.radians(self.cur_w),2), float(self.heading) 
