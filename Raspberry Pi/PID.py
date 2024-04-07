import time
import math


class PID():
    def __init__(self,Kp: float,Ki: float,Kd: float):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.total_error = 0.0
        self.integral = 0.0
        

    def Calculate(self, input: float, sensor_reading: float, end_cond = 0.5, end_limit = 0.5, debug=False):

        self.setpoint = input
        measured_val = sensor_reading
        end_flag = False
        # calculates error
        error = float(self.setpoint - measured_val)

        # Proportional
        proportional = self.Kp * error

        # Integral 
        self.total_error += error
        self.integral = (self.Ki * self.total_error)
        # limit integral to prevent windup
        self.integral = max(min(self.integral, abs((input+0.1)*20)), -abs((input+0.1)*20))
        # Integral windup
        if error>=-end_limit and error<=end_limit:
            self.total_error = 0.0
            self.integral = 0.0

        # Derivative   
        derivative = self.Kd * (error - self.prev_error)

        PIDoutput = proportional + self.integral + derivative
        self.prev_error = error

        # End condition if error is low, stop PID
        if error <= end_cond and error >= -end_cond:
            error = 0
            end_flag = True
        else:
            end_flag = False
        
        if debug:
            print(f"{proportional} | {self.integral} | {derivative} Status:{end_flag}")
        
        return float(PIDoutput), float(self.integral), bool(end_flag)
    
    def Reset(self):
        self.setpoint = 0
        self.prev_error = 0
        self.total_error = 0
        self.newvalue = 0
        self.integral = 0
        
