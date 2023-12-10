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
import IMU

# Variables & Objects
enc1 = Encoder.Encoder(4,5,20.0)
enc2 = Encoder.Encoder(6,12,20.0)
enc3 = Encoder.Encoder(13,16,20.0)
enc4 = Encoder.Encoder(17,18,20.0)
Robot = Odometry.Mecanum_Drive(enc1,enc2,enc3,enc4)
Robot_IMU = IMU.Mpu()
end_flag = False
init_yaw = False
# Encoders
E1 = 0.0
E2 = 0.0 
E3 = 0.0
E4 = 0.0
# Velocities
V1 = 0.0
V2 = 0.0
V3 = 0.0
V4 = 0.0
# Robot Odometry
cur_x = 0.0
cur_y = 0.0
cur_w = 0.0 # Z distance
theta = 0.0 # heading
imu_x = 0.0
imu_y = 0.0
imu_w = 0.0
# Fused Odometry
# IMU Data
IMU_Val = (0.0,0.0,0.0,0.0,0.0,0.0)
# Robot Position
Robot_x = 0.0
Robot_y = 0.0
# PID
pidx = PID.PID(10,0.1,2)
pidy = PID.PID(7.5,0.1,5)
pidw = PID.PID(14,2,5)
timeint = 0
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4,V1,V2,V3,V4, IMU_Val, init_yaw, timeint, Robot_x,Robot_y
    
    while True:
        cur_time = time.time()
        # Serial comm
        if ser.in_waiting >=24:
            imudata = ser.read(24)
            IMU_Val = struct.unpack('ffffff',imudata)
            
        if timeint>10:
            if not init_yaw:
                Robot_IMU.setInitialYaw(IMU_Val[2])
                print("Yaw Calibrated")
                print(IMU_Val[2])
            init_yaw = True
            
        
        # IMU
        pose, theta = Robot_IMU.getOdometry(IMU_Val,cur_time)
        # Odometry
        cur_x = Robot.getxDist()
        cur_y = Robot.getyDist()
        cur_w = pose +0.15
        E1 = enc1.getDist()
        E2 = enc2.getDist()
        E3 = enc3.getDist()
        E4 = enc4.getDist()
#         V1 = enc1.getVel()
#         V2 = enc2.getVel()
#         V3 = enc3.getVel()
#         V4 = enc4.getVel()
#         _x_, _y_, cur_w, theta = Robot.CalculateOdometry(cur_time)
        Robot_x, Robot_y = UpdatePosition(cur_x,cur_y,theta)
#         print("{} :{}: {}:: {}".format(cur_x,cur_y,cur_w,theta))
        print("{} : {} : {} :{} : {}".format(Robot_x,Robot_y,cur_x,cur_y,theta))
        # Fused Odometry
        # print("{}:{}:{}::{}".format(E1,E2,E3,E4))
#         print("{}:{}:{}::{}".format(V1,V2,V3,V4))
#         print("{}||{}".format(theta,pose))
#         print("{}+{}-{}-{}={}".format(E1,E2,E3,E4,E1+E2-E3-E4))
#         print(IMU_Val)
        if not init_yaw:   
            timeint += 1
        else:
            timeint = 0
        #print("Roll:{:.2f} Pitch:{:.2f} Yaw:{:.2f}".format(Roll,Pitch,Yaw))
        time.sleep(0.05)

def UpdatePosition(x,y,theta):
    t = math.radians(theta)
    r_x = math.cos(t)*x + math.sin(t)*y
    r_y = -math.sin(t)*x + math.cos(t)*y
    
    return round(r_x,2),round(r_y,2)
def SendData(Vx,Vy,Wz):
    try:
        data = struct.pack('fff', Vx, Vy, Wz)
        ser.write(data)
        #print(data)
    except serial.SerialException as e:
        print(f"Error writing data to serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def map_values(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * float(outMax - outMin))

def PID_Controller(x,y,w):
    global cur_x, cur_y, cur_w, pidx, pidy, pidw
    Vx = 0
    Vy = 0
    Wz = 0
    end_Flag = False
    x_val,i1,endx = pidx.Calculate(x,cur_x,5)
    y_val,i2,endy= pidy.Calculate(y,cur_y,5)
    w_val,i3,endw = pidw.Calculate(w,cur_w,5,0.02,time.time())
    x_limit = 20 if x_val < 20 else (abs(x)*(8+0.1+2)+1)
    y_limit = 20 if y_val < 20 else (abs(y)*(8+0.1+5)+1)
    w_limit = 20 if w_val < 20 else (abs(w)*(15+0.1+2)+1)
    if x_val != 0:
        Vx = map_values(x_val,-x_limit,x_limit,-0.3768,0.3768)
        Vx = max(min(Vx, 0.3768), -0.3768)
    if y_val != 0:
        Vy = map_values(y_val,-y_limit,y_limit,-0.3768,0.3768)
        Vy = max(min(Vy, 0.3768), -0.3768)
    if w_val != 0:
        Wz = map_values(w_val,-w_limit,w_limit,-4,4)
        Wz = max(min(Wz, 4), -4)
#     print("{}".format(endx))
#     print(i3)
#     print("{} {} {}".format(endx,endy,endw))
    if endx and endy:
        end_Flag = True
    SendData(Vx,Vy,Wz)
    return end_Flag

        
def MoveRobot(type, dist, speed):
    global cur_x, cur_y, cur_w
    v = speed
    setpoint = dist
    x = cur_x
    y = cur_y
    w = cur_w
    if type == 0:
        x = setpoint + cur_x
    elif type == 1:
        y = setpoint + cur_y
    elif type == 2:
        w = setpoint + cur_w                                              
        
    while True:
        flag = PID_Controller(x,y,w)
        if flag:
            print("done")
            break
    SendData(0,0,0)
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
       
        
def MoveToCoord(target_x, target_y):
    global theta, end_flag
    angle = math.radians(theta)
    x = 0
    y = 0
    w = cur_w
    x =  math.cos(angle)*target_x - math.sin(angle)*target_y
    y =  math.sin(angle)*target_x + math.cos(angle)*target_y
    pid_end_flag = False
    while not end_flag:
        flag = PID_Controller(x,y,w)
        if flag:
            print("done")
            break
    SendData(0,0,0)
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    
        
if __name__ == '__main__':
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    ser.reset_input_buffer()
    update_odometry_thread = threading.Thread(target=updateOdometry)
    update_odometry_thread.daemon = True
    update_odometry_thread.start()
    #initialize yaw
    #Robot_IMU.setInitialYaw(IMU_Val[2])
    time.sleep(3)
    try:
        while True:
            if not end_flag:
    #                         MoveRobot(2,2*math.pi,0.2)
                MoveRobot(0,40,0.3)
                #MoveToCoord(40,0)
            
#                 time.sleep(3)
#             if not end_flag:
#                 MoveRobot(1,-30,0.3)
#                 MoveToCoord(15,15)
                
                print("completed seq1")
#                 time.sleep(3)
                
#                 MoveRobot(0,-30,0.25)
#                 MoveToCoord(20,30)
                
                time.sleep(2)
#                 MoveRobot(1,30,0.3)
# #                 MoveRobot(2,math.pi,0.2)
# #                 
#                 time.sleep(2)
#                 
#                 MoveToCoord(0,0)
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

