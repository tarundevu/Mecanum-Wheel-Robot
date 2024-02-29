#!/usr/bin/env python3
import time
import serial
import struct
import math
import copy
# import pygame
import RPi.GPIO as GPIO
import Encoder
import Odometry
import threading
import PID
import IMU
import Astar

# Map
StartPos = (16,16)
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
IMU_Val = (0.0,0.0,0.0)
# Robot Position
Robot_x = 0.0
Robot_y = 0.0
# PID
pidx = PID.PID(10,0.1,8)
pidy = PID.PID(10,0.1,8)
pidw = PID.PID(2,0.01,7)
timeint = 0
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4,V1,V2,V3,V4, IMU_Val, init_yaw, timeint, Robot_x,Robot_y
    
    while True:
        cur_time = time.time()
        end_time = 0.0
        # Serial comm
        if ser.in_waiting >=12:
            imudata = ser.read(12)
            IMU_Val = struct.unpack('fff',imudata)
            end_time = time.time()
#             print("ok")
        if timeint>10:
            if not init_yaw:
                Robot_IMU.setInitialYaw(IMU_Val[2])
                print("Yaw Calibrated")
                print(IMU_Val[2])
            init_yaw = True
            
        
        # IMU
        pose, theta = Robot_IMU.getOdometry(IMU_Val,cur_time)
        # Odometry
        cur_x = Robot.getxDist() + StartPos[0]
        cur_y = Robot.getyDist() + StartPos[1]
        cur_w = pose 
#         E1 = enc1.getDist()
#         E2 = enc2.getDist()
#         E3 = enc3.getDist()
#         E4 = enc4.getDist()
#         V1 = enc1.getVel()
#         V2 = enc2.getVel()
#         V3 = enc3.getVel()
#         V4 = enc4.getVel()
#         _x_, _y_, cur_w, theta = Robot.CalculateOdometry(cur_time)
        Robot_x, Robot_y = UpdatePosition(cur_x,cur_y,theta)
#         print("{} :{}: {}:: {q}".format(cur_x,cur_y,cur_w,theta))
#         print(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {theta}")
#         print("The time of execution of above program is :",(end_time-cur_time) * 10**3, "ms")
        # Fused Odometry
#                                                                                                                                                                print("{}:{}:{}::{}".format(E1,E2,E3,E4))
#         print("{}:{}:{}::{}".format(E1,E2,E3,E4))
#         print("{}||{}".format(theta,pose))
#         print("{}+{}-{}-{}={}".format(E1,E2,E3,E4,E1+E2-E3-E4))
#         print(IMU_Val)
        if not init_yaw:   
            timeint += 1
        else:
            timeint = 0
        
        time.sleep(0.05)

def UpdatePosition(x,y,theta):
    rad = math.radians
    cos = math.cos
    sin = math.sin
    t = rad(theta)
    r_x = cos(t)*x + sin(t)*y
    r_y = -sin(t)*x + cos(t)*y
    
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

    Vx = 0
    Vy = 0
    Wz = 0
    end_Flag = False 
    x_diff = x - cur_x
    y_diff = y - cur_y
    w_diff = w - cur_w
    while not end_Flag:
        x_val,i1,endx = pidx.Calculate(x,cur_x,(x+4)*0.75,'x')
        y_val,i2,endy = pidy.Calculate(y,cur_y,(y+4)*0.75,'y')
        w_val,i3,endw = pidw.Calculate(w,cur_w,math.pi/4,'w',0.02,0.02)
        x_limit = abs(x_diff+0.01)*20
        y_limit = abs(y_diff+0.01)*20
        w_limit = abs(w_diff+0.001)*10
        x_spd_lim = 0.1 if abs(x_diff)<30 else 0.2
        y_spd_lim = 0.1 if abs(y_diff)<30 else 0.2 
        w_spd_lim = 0.5 if abs(w)<=math.pi/2 else 2.5
        if x_val != 0:
            Vx = map_values(x_val,-x_limit,x_limit,-x_spd_lim,x_spd_lim)
            Vx = max(min(Vx, x_spd_lim), -x_spd_lim)
        if y_val != 0:
            Vy = map_values(y_val,-y_limit,y_limit,-y_spd_lim,y_spd_lim)
            Vy = max(min(Vy, y_spd_lim), -y_spd_lim)
        if w_val != 0:
#             print(w_limit,-w_limit)
            Wz = map_values(w_val,-w_limit,w_limit,-w_spd_lim,w_spd_lim)
            Wz = max(min(Wz, w_spd_lim), -w_spd_lim)
            
#         print("{} {} {}".format(i1,i2,i3))
#         print("{} {} {}".format(endx,endy,endw))
#         print("{} {} {}".format(x,y,w))
#         print("{} {} {}".format(Vx,Vy,Wz))
#         print(f"Coord: {cur_x} {cur_y}")
#         print("limits:{} {} {}".format(round(x_limit,2),round(y_limit,2),round(w_limit,2)))
#         print("val:{} {} {}".format(round(x_val,2),round(y_val,2),round(w_val,2)))

        

        SendData(Vx,Vy,Wz)
        if endx and endy and endw:
            end_Flag = True
    print("*********PID DONE***********")
    return end_Flag

        
def MoveRobot(type, dist):
    
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
        
        
#     while True:
    PID_Controller(x,y,w)
#         if flag:
#             print("done")
#             break
    SendData(0,0,0)
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
       
        
def MoveToCoord(target_x, target_y,init_w = 0):
#     global theta, end_flag
    angle = math.radians(theta)
    x = 0
    y = 0
    w = init_w if init_w != 0 else cur_w
    x =  math.cos(angle)*target_x - math.sin(angle)*target_y
    y =  math.sin(angle)*target_x + math.cos(angle)*target_y
    
    PID_Controller(x,y,w)
#         print(w)
    print("*********************MoveToCoord Seq complete********************")
    SendData(0,0,0)
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()

def Move_Astar(target_x,target_y):
    start = (Robot_x,Robot_y)
    path = Astar.main(start,(target_x,target_y))
    init_w = copy.copy(cur_w)
    try:
        while not end_flag:
            for x,y in path:
                MoveToCoord(x,y,init_w)
                print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
                path.remove((x,y))
                time.sleep(0.01)
            
            break
    except:
        print("No path found!")
    print("******************Astar complete*******************")
        
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
    #               MoveRobot(2,2*math.pi,0.2)
#                 MoveRobot(2,0.25*math.pi)
#                 time.sleep(3)
#                 MoveToCoord(16,16)
#                 MoveToCoord(60,60)
#                 MoveToCoord(72,44)
#                 MoveToCoord(126,44)
#                 MoveToCoord(140,60)
#                 SendData(30,0,0)
#                 time.sleep(4)
# #             if not end_flag:
#                 MoveRobot(0,2) 
                Move_Astar(75,100)
                time.sleep(2)
                Move_Astar(75,25)
                
                
#                 print("astar****************")

                end_flag=True
                
#
            
            time.sleep(0.5)
#             window_surface.blit(background, (0, 0))
    except KeyboardInterrupt:
        SendData(0,0,0)
        print("Exiting...")
        
    finally:
        GPIO.cleanup()