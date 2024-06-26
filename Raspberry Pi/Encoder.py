import time
import math
import RPi.GPIO as GPIO

class Encoder():
    '''
        This class is used to calculate the no. of rotations from the encoders
    '''
    def __init__(self, pin_a: int, pin_b: int, pulses_per_revolution: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.pulses_per_revolution = pulses_per_revolution
        self.prev_count = 0
        self.prev_time = 0
        self.vel_history = []
        self.vel_window = 4
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.FALLING, callback=self.Encoder_ISR, bouncetime=2)
        
    def Encoder_ISR(self, channel):
        if (GPIO.input(self.pin_b) == GPIO.LOW):
            self.count += 1
        else:
            self.count -= 1
            
    def getCount(self)->int:
        return self.count
    
    def getDistancePerPulse(self)->float:
        '''Returns distance per pulse in centimetres '''
        distance_per_pulse = math.pi*6/self.pulses_per_revolution
        return distance_per_pulse
    
    def getDist(self):
        dist = self.getDistancePerPulse * self.getCount
        return dist
    
    def getVel(self,cur_time)->float:
        ''' V = (Encoder count*Distance per pulse(in m))/time elapsed ,
            ω = V*wheel_radius(in m)
        '''
        encoder_count = self.getCount() - self.prev_count # gets the encoder count per sec
        dT = cur_time - self.prev_time
        linear_vel = (encoder_count * self.getDistancePerPulse()/100)/dT
        angular_vel = linear_vel * 0.03
        

        # Moving average filter - in order to compensate for low encoder resolution
        self.vel_history.append(angular_vel)
        if self.vel_history > self.vel_window:
            self.vel_history.pop(0) # remove oldest value
        moving_avg = 0.0
        if self.vel_history:
            moving_avg = sum(self.vel_history)/len(self.vel_history) # add the average value

        self.prev_count = self.getCount()
        self.prev_time = cur_time
        
        return float(moving_avg)
        
