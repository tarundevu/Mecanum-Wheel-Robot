import time
import math
import Encoder
# import Main
import RPi.GPIO as GPIO

class Mecanum_Drive():
    def __init__(self,E1,E2,E3,E4):
        pass
        self.x_dist = 0
        self.y_dist = 0
        self.w_dist = 0
        self.Vx = 0
        self.Vy = 0
        self.Vw = 0
        self.Enc1 = E1
        self.Enc2 = E2
        self.Enc3 = E3
        self.Enc4 = E4

    def getxDist(self):
        x = (1/4.0)*(-self.Enc1.getDist()+self.Enc2.getDist()+self.Enc3.getDist()-self.Enc4.getDist())
        x = format(x,".2f")
        #self.x_dist = x
        return float(x)
    def getyDist(self): 
        y = (1/4.0)*(self.Enc1.getDist()+self.Enc2.getDist()+self.Enc3.getDist()+self.Enc4.getDist())
        y = format(y,".2f")
        #self.y_Dist = y
        return float(y)
    def getzDist(self, E1,E2,E3,E4,cur_w):
        # theta = (1/51.6)*(self.Enc1.getDist()-self.Enc2.getDist()+self.Enc3.getDist()-self.Enc4.getDist())
        delta_FL = self.Enc1.getDist() - E1
        delta_RL = self.Enc3.getDist() - E2
        delta_FR = self.Enc2.getDist() - E3
        delta_RR = self.Enc4.getDist() - E4

        delta_L = (delta_FL + delta_RL) / 2
        delta_R = (delta_FR + delta_RR) / 2

        delta_theta = (delta_R - delta_L) / 11.5
        theta = cur_w + delta_theta
        theta = math.atan2(math.sin(theta), math.cos(theta))
        theta = format(theta,".2f")
        return float(theta)
    def getVel(self):
        
        pass
