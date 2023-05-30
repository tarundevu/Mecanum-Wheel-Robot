import time
import math
import Encoder

class PID():
    def __init__(self,Kp,Kd,Ki):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = 0
        self.error = 0
        self.prev_error = 0
        self.newvalue = 0
        # self.prev_time = 0
        self.integral = 0
        

    def Calculate(self,val,sensor_val):
        
        self.setpoint = val
        measured_speed = sensor_val

        error = self.setpoint - measured_speed

        proportional = self.Kp * self.error
        self.integral += self.Ki * self.error
        derivative = self.Kd * (self.error - self.prev_error)

        change = proportional + self.integral + derivative
        new_value = change
        self.prev_error = error
    
        return float(new_value)
    
    def reset(self):
        self.setpoint = 0
        self.error = 0
        self.prev_error = 0
        self.newvalue = 0
        # self.prev_time = 0
        self.integral = 0