#!/usr/bin/env python3
import time
import serial
import struct
import math
# import pygame
import RPi.GPIO as GPIO
import Encoder
import Odometry
import threading
import PID

# Define GPIO pins

# Initialize GPIO

# Variables

enc1 = Encoder.Encoder(4,5,20.0)
enc2 = Encoder.Encoder(6,12,20.0)
enc3 = Encoder.Encoder(13,16,20.0)
enc4 = Encoder.Encoder(17,18,20.0)
Robot = Odometry.Mecanum_Drive(enc1,enc2,enc3,enc4)
end_flag = False
E1 = 0.0
E2 = 0.0 
E3 = 0.0
E4 = 0.0
V1 = 0.0
V2 = 0.0
V3 = 0.0
V4 = 0.0
cur_x = 0.0
cur_y = 0.0
cur_y2 = 0.0
cur_w = 0.0
theta = 0.0
temp = 0
# PID
pidx = PID.PID(1,0.1,0)
pidy = PID.PID(1,0.1,0)
pidw = PID.PID(5,0.1,0)
# pid4 = PID.PID(1,0,0)
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4,V1,V2,V3,V4
    while True:
        cur_time = time.time()
        cur_x = Robot.getxDist()
        cur_y = Robot.getyDist()
        # cur_w = Robot.getzDist(E1,E2,E3,E4,cur_w)
#         E1 = enc1.getDist()
#         E2 = enc2.getDist()
#         E3 = enc3.getDist()
#         E4 = enc4.getDist()
#         V1 = enc1.getVel()
#         V2 = enc2.getVel()
#         V3 = enc3.getVel()
#         V4 = enc4.getVel()
        _x_, _y_, cur_w, theta = Robot.CalculateOdometry(cur_time)
#         print("{}:{}:{}::{}".format(cur_x,cur_y,cur_w,theta))
#         print("{}:{}:{}::{}".format(E1,E2,E3,E4))
#         print("{}:{}:{}::{}".format(V1,V2,V3,V4))
#         print("{} : {}".format(cur_y2,cur_y))
#         print("{}+{}-{}-{}={}".format(E1,E2,E3,E4,E1+E2-E3-E4))
        time.sleep(0.1)

def SendData(Vx,Vy,Wz):
    data = struct.pack('fff', Vx, Vy, Wz)
    ser.write(data)
    #print(data)
    
def vel_to_wheelspeed(Vx,Vy,Wz):
    radius = 0.03
    lx = 0.068
    ly = 0.061
    w1 = 1/radius * (Vy + Vx +(lx + ly)*Wz)
    w2 = 1/radius * (Vy - Vx -(lx + ly)*Wz)
    w3 = 1/radius * (Vy - Vx +(lx + ly)*Wz)
    w4 = 1/radius * (Vy + Vx -(lx + ly)*Wz)
    return w1,w2,w3,w4

def wheelspeed_to_vel(w1,w2,w3,w4):
    radius = 0.03
    lx = 0.068
    ly = 0.061
    Vx = (w1-w2-w3+w4)*radius/4.0
    Vy = (w1+w2+w3+w4)*radius/4.0
    Wz = (w1-w2+w3-w4)*radius/(4.0*(lx + ly))
    return Vx,Vy,Wz

def map_values(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * float(outMax - outMin))

def PID_Controller(x,y,w):
    global cur_x, cur_y, cur_w, pidx, pidy, pidw
    Vx = 0
    Vy = 0
    Wz = 0
    x_val = pidx.Calculate(x,cur_x,5)
    y_val = pidy.Calculate(y,cur_y,5)
    w_val = pidw.Calculate(w,cur_w,1)
    x_limit = 20 if x_val < 20 else (x+50)
    y_limit = 20 if y_val < 20 else (y+50)
    w_limit = 20 if w_val < 20 else (w+50)
    if x_val != 0:
        Vx = map_values(x_val,-x_limit,x_limit,-0.3768,0.3768)
    if y_val != 0:
        Vy = map_values(y_val,-y_limit,y_limit,-0.3768,0.3768)
    if w_val != 0:
        Wz = map_values(w_val,-w_limit,w_limit,-4,4)
    print("{}".format(x_val))
#     print("{} {} {}".format(Vx,Vy,Wz))
    SendData(Vx,Vy,Wz)
        
def MoveRobot(type, dist, speed):
    global cur_x, cur_y, cur_w
    v = speed
    setpoint = dist
    x = 0
    y = 0
    w = 0
    if type == 0:
        x = setpoint + cur_x
    elif type == 1:
        y = setpoint + cur_y
    elif type == 2:
        w = setpoint + cur_w
        
    while True:
        PID_Controller(x,y,w)
        

if __name__ == '__main__':
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    ser.reset_input_buffer()
    update_odometry_thread = threading.Thread(target=updateOdometry)
    update_odometry_thread.daemon = True
    update_odometry_thread.start()
    
    try:
        while True:
            if not end_flag:
                while True:
                    if not end_flag:
                        MoveRobot(0,30,0.2)
                        end_flag=True
#                     SendData(0,0.08,0)
#             time.sleep(2)
#             if not end_flag:
#                 MoveRobot(0,60,0.35)
#                 end_flag=False
#             time.sleep(2)
#             if not end_flag:
#                 MoveRobot(1,-60,0.35)
#                 end_flag=False
#             time.sleep(2)
#             if not end_flag:
#                 MoveRobot(0,-60,0.35)
#                 end_flag=True
            
            time.sleep(0.5)
#             window_surface.blit(background, (0, 0))
    except KeyboardInterrupt:
        SendData(0,0,0)
        print("Exiting...")
        
    finally:
        GPIO.cleanup()
