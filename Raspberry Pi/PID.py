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
        self.prev_time = 0.0
        self.rr_time = 0.0
        

    def Calculate(self, input: float, sensor_reading: float, cur_time: float, end_cond = 0.5, int_range = 15, int_windup = 0.5, debug = False):

        end_flag = False
        self.setpoint = input
        proportional = 0
        derivative = 0
        measured_val = sensor_reading
        
        # calculates error
        error = self.setpoint - measured_val
        dT = cur_time - self.prev_time
        
        # Proportional
        proportional = self.Kp * error
        
        if(dT>0.01):
            # Integral
            if abs(error)<int_range:
                self.integral = (self.Ki * self.total_error)
                # limit integral to prevent windup
                self.integral = max(min(self.integral, 1000), -1000)
                # Integral windup
                if abs(error)<int_windup:
                    self.total_error = 0.0
                    self.integral = 0.0
                self.total_error += error
            else:
                self.integral = 0
            
            self.prev_time = cur_time
        
        # Derivative   
        derivative = self.Kd * (error - self.prev_error)
        
        PIDoutput = proportional + self.integral + derivative

        self.prev_error = error

        
        # End condition if error is low, stop PID
        if abs(error)<end_cond:
            error = 0
            end_flag = True
        else:
            end_flag = False
        
        if debug:
            print(f"{proportional:.2f} | {self.integral:.2f} | {derivative:.2f} Status:{end_flag}")
        
#         time.sleep(0.015)
        return float(PIDoutput), bool(end_flag)
    
    def Reset(self):
        self.setpoint = 0
        self.prev_error = 0
        self.total_error = 0
        self.newvalue = 0
        self.integral = 0
        self.rr_time = 0
        self.prev_time = 0
        

