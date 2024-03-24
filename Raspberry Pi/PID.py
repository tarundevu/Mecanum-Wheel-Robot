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
        self.integral = 0
        

    def Calculate(self,val,sensor_val,end_con = 0.5,end_limit = 0.5,debug=False):
        start_time = time.time()
        self.setpoint = val
        measured_val = sensor_val
        end_flag = False
        # calculates error
        error = float(self.setpoint - measured_val)

        # Proportional
        proportional = self.Kp * error

        # Integral 
        self.total_error += error
        self.integral = (self.Ki * self.total_error)
        # limit integral to prevent windup
        self.integral = max(min(self.integral, abs((val+0.1)*10)), -abs((val+0.1)*10))
        # integral windup
        if error>=-end_limit and error<=end_limit:
            self.total_error = 0
            self.integral = 0

        # Derivative   
        derivative = self.Kd * (error - self.prev_error)

        new_value = proportional + self.integral + derivative
        self.prev_error = error

        # end condition if error is low, stop PID
        if error<=0.02 and error>=-0.02:
            error = 0
            end_flag = True
        else:
            end_flag = False
        
        end_time = time.time()
        # print(start_time-end_time)
        if debug:
            print(f"{proportional} {self.integral} {derivative} Status:{end_flag}")
        
        return float(new_value), float(self.integral), bool(end_flag)
    
    def Reset(self):
        self.setpoint = 0
        self.error = 0
        self.prev_error = 0
        self.total_error = 0
        self.newvalue = 0
        self.integral = 0
        
