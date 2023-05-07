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
        x = (3/4.0)*(-self.Enc1.getDist()+self.Enc2.getDist()+self.Enc3.getDist()-self.Enc4.getDist())
        x = format(x,".2f")
        #self.x_dist = x
        return float(x)
    def getyDist(self): 
        y = (3/4.0)*(self.Enc1.getDist()+self.Enc2.getDist()+self.Enc3.getDist()+self.Enc4.getDist())
        y = format(y,".2f")
        #self.y_Dist = y
        return float(y)
    def getzDist(self):
        theta = (3/6.5)*(self.Enc1.getDist()-self.Enc2.getDist()+self.Enc3.getDist()-self.Enc4.getDist())
        return float(theta)
    