import time
import math
import logging


class PID():
    def __init__(self,Kp: float,Ki: float,Kd: float):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.total_error = 0.0
        self.integral = 0.0
        

    def Calculate(self, input: float, sensor_reading: float,cur_time: float, end_cond = 0.5, end_limit = 0.5, int_limit = 20, debug = False):

        end_flag = False
        self.setpoint = input
        measured_val = sensor_reading
        prev_time = 0.0
        # calculates error
        error = self.setpoint - measured_val
        dT = cur_time - prev_time
        # Proportional
        proportional = self.Kp * error

        # Integral 
        self.integral = (self.Ki * self.total_error)*dT
        # limit integral to prevent windup
        self.integral = max(min(self.integral, abs((input+0.1)*int_limit)), -abs((input+0.1)*int_limit))
        # Integral windup
        if error>=-end_limit and error<=end_limit:
            self.total_error = 0.0
            self.integral = 0.0

        # Derivative   
        derivative = self.Kd * (error - self.prev_error)/dT

        PIDoutput = proportional + self.integral + derivative

        self.prev_error = error
        self.total_error += error
        prev_time = cur_time
        # End condition if error is low, stop PID
        if error <= end_cond and error >= -end_cond:
            error = 0
            end_flag = True
        else:
            end_flag = False
        
        if debug:
            logging.debug(f"{proportional} | {self.integral} | {derivative} Status:{end_flag}")
        
        return float(PIDoutput), bool(end_flag)
    
    def Reset(self):
        self.setpoint = 0
        self.prev_error = 0
        self.total_error = 0
        self.newvalue = 0
        self.integral = 0
        
