import time
import RPi.GPIO as GPIO
#/*
# This class is used to calculate the no. of rotations from the Outputs of photo interruptor modules
# 
# */
class Encoder():
    def __init__(self, pin_a, pin_b, pulses_per_revolution):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.pulses_per_revolution = pulses_per_revolution
#         self.Astate = GPIO.input(self.pin_a)
#         self.Bstate = GPIO.input(self.pin_b)
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
    
    def getRev(self):
        rev = self.count/self.pulses_per_revolution
        return rev
    
    def getDist(self):
        dist = (self.count/self.pulses_per_revolution)*18.84
        return dist
        
