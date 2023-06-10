import time
import math


class PID():
    def __init__(self,Kp,Ki,Kd,limit = 0):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = 0
        self.error = 0
        self.prev_error = 0
        self.total_error = 0
        self.newvalue = 0
        self.prev_time = 0
        self.integral = 0
        self.limit = limit
        

    def Calculate(self,val,sensor_val,int_limit):
        
        self.setpoint = val
        measured_speed = sensor_val
        # calculates error
        error = self.setpoint - measured_speed
        # proportional
        proportional = self.Kp * error
        # integral only
        if error>-int_limit and error<int_limit:
            self.total_error += error
            self.integral = (self.Ki * self.total_error)
            # limit integral
            if self.integral>(val*0.5):
                self.integral = (val*0.5)
            elif self.integral < -(val*0.5):
                self.integral = -(val*0.5)
            # if error is low stop integral
            if error>-0.5 and error<0.5:
                self.total_error = 0
        else:
            self.integral = 0
        derivative = self.Kd * (error - self.prev_error)

        change = proportional + self.integral + derivative
        new_value = change
        self.prev_error = error
#         print("{} + {} + {} = {}".format(proportional,self.integral,derivative,new_value))
        return float(new_value)