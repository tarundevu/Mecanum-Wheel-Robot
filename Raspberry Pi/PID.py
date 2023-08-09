import time
import math


class PID():
    def __init__(self,Kp,Ki,Kd):
        
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
        

    def Calculate(self,val,sensor_val,int_limit,end_limit = 0.5):
        
        self.setpoint = val
        measured_speed = sensor_val
        end_flag = False
        # calculates error
        error = float(self.setpoint - measured_speed)
        # proportional
        proportional = self.Kp * error
        # integral only
        if error>=-int_limit and error<=int_limit and self.Ki!=0:
            self.total_error += error
            self.integral = (self.Ki * self.total_error)
            # limit integral
            self.integral = max(min(self.integral, val*0.7), -abs(val*0.7))
            # if error is low stop integral
            if error>=-end_limit and error<=end_limit:
                self.total_error = 0

        else:
            self.integral = 0
        derivative = self.Kd * (error - self.prev_error)

        change = proportional + self.integral + derivative
        new_value = change
        self.prev_error = error

        if (error - self.prev_error) > -1 or (error - self.prev_error) < 1:
            end_flag = True
        string = str(proportional) +""+ str(self.integral) +""+ str(derivative)
#         print("{} + {} + {} = {}".format(proportional,self.integral,derivative,new_value))
        return float(new_value), string, end_flag
    
