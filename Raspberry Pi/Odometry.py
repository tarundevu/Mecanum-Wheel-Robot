import time
import math
import Encoder
import RPi.GPIO as GPIO

class Mecanum_Drive():
    def __init__(self,E1: object,E2: object,E3: object,E4: object):
        self.prev_time = 0
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

    def getxDist(self)->float:
        '''
        Returns the x-distance moved using encoder
        '''
        x = (1/4.0)*(self.Enc1.getDist()-self.Enc2.getDist()-self.Enc3.getDist()+self.Enc4.getDist())
        x = format(x,".2f")

        return float(x)
    
    def getyDist(self)->float: 
        '''
        Returns the y-distance moved using encoder
        '''
        y = (1/4.0)*(self.Enc1.getDist()+self.Enc2.getDist()+self.Enc3.getDist()+self.Enc4.getDist())
        y = format(y,".2f")

        return float(y)
    
    def getFusedOdometry(self, val1: tuple, val2: tuple)->tuple:
        fused_data = []
        alpha = 0.98 #complimentary filter value
        fused_data[0] = alpha *  (val1[0]) + (1-alpha)*val2[0]
        fused_data[1] = alpha *  (val1[1]) + (1-alpha)*val2[1]
        
        return tuple(fused_data)
   