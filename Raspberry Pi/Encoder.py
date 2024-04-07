import time
import math
import RPi.GPIO as GPIO # type: ignore
'''
 This class is used to calculate the no. of rotations from the Outputs of encoders
'''
class Encoder():
    def __init__(self, pin_a: int, pin_b: int, pulses_per_revolution: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.pulses_per_revolution = pulses_per_revolution
        self.prev_count = 0
        self.prev_time = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.FALLING, callback=self.Encoder_ISR, bouncetime=2)
        
    def Encoder_ISR(self, channel):
        if (GPIO.input(self.pin_b) == GPIO.LOW):
            self.count += 1
        else:
            self.count -= 1
            
    def getCount(self):
        return self.count
    
    def getDistancePerPulse(self):
        distance_per_pulse = math.pi*6/self.pulses_per_revolution
        return distance_per_pulse
    
    def getDist(self):
        dist = self.getDistancePerPulse * self.getCount
        return dist
    
    def getVel(self,cur_time):
        encoder_count = self.getCount() - self.prev_count # gets the encoder count per sec
        dT = cur_time - self.prev_time
        linear_vel = (encoder_count * self.getDistancePerPulse()/100)/dT
        angular_vel = linear_vel * 0.03
        # rpm  = (encoder_count*60.0)/self.pulses_per_revolution # finds the rpm
        # angular_vel = rpm * 0.10472 # finds angular velocity
        # # linear_vel = angular_vel * 0.03 # finds the linear velocity
        # angular_vel = format(angular_vel,".2f")
        self.prev_count = self.getCount()
        self.prev_time = cur_time
        return float(angular_vel)
        
