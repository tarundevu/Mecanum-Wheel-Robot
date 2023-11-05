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
        pose = [0.0,0.0,0.0]
        yaw = data[2]
#         accx = data[3]
#         accy = data[4]
        ### heading, W ###
        calibrated_yaw = yaw + 6.71+3.57 -11 -3.73
        change =  yaw - self.prev_yaw

        if change > 180:
            change -= 360
        elif change < -180:
            change += 360
       
        self.cur_w += change

        self.heading = round(calibrated_yaw,2)
#         self.cur_w = math.radians(self.cur_w)
#         self.cur_w = math.degrees(cur_w_rad)
        ### X and Y ###
#         timeelapsed = cur_time - self.prev_time
#         
#         dist_x =  accx * timeelapsed*timeelapsed
#         dist_y =  accy * timeelapsed*timeelapsed
# 
#         self.cur_x += dist_x
#         self.cur_y += dist_y
#         
#         pose[0] = round(self.cur_x,2)
#         pose[1] = round(self.cur_y,2)
#         pose[2] = round(math.radians(self.cur_w),2)
        
        self.prev_yaw = yaw
        self.prev_time = cur_time

        return round(math.radians(self.cur_w),2), float(self.heading) 
