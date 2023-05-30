import time
import math
import Encoder
# import Main
import RPi.GPIO as GPIO

class Mecanum_Drive():
    def __init__(self,E1,E2,E3,E4):
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

    def getxDist(self):
        x = (1/4.0)*(self.Enc1.getDist()-self.Enc2.getDist()-self.Enc3.getDist()+self.Enc4.getDist())
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
        delta_FL = self.Enc1.getDist() 
        delta_RL = self.Enc3.getDist()
        delta_FR = self.Enc2.getDist()
        delta_RR = self.Enc4.getDist()

#         delta_L = (delta_FL + delta_RL) / 2
#         delta_R = (delta_FR + delta_RR) / 2

        delta_theta = (-(delta_FR + delta_RR)+(delta_FL + delta_RL)) / 4.0
        theta = delta_theta
        if theta >= 3.141:
            theta = theta - 3.141*2
        elif theta <= -3.141:
            theta = theta + 3.141*2
        else:
            theta = theta + 0
#         theta = math.atan2(math.sin(theta), math.cos(theta))
        theta = format(theta,".2f")
        return float(theta)
    
    def CalculateOdometry(self,current_time):
        radius = 0.03
        lx = 0.068
        ly = 0.061

        timeelapsed = current_time - self.prev_time
        E1, E2, E3, E4 = self.Enc1.getVel(),self.Enc2.getVel(),self.Enc3.getVel(),self.Enc4.getVel()
        Vx = (E1-E2-E3+E4)*radius/4.0
        Vy = (E1+E2+E3+E4)*radius/4.0
        Wz = (E1-E2+E3-E4)*radius/(4.0*(lx + ly))
        cur_x = Vx * timeelapsed *100
        cur_y = Vy * timeelapsed *100
        cur_w = Wz * timeelapsed
        
        self.x_dist += cur_x
        self.y_dist += cur_y
        self.w_dist += cur_w

        heading = self.w_dist
        if heading > 3.14:
            heading = heading - math.pi*2
        elif heading < -3.14:
            heading = heading + math.pi*2
   
        self.prev_time = current_time
        return float(self.x_dist), float(self.y_dist), float(self.w_dist), float(heading)