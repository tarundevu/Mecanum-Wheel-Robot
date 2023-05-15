#!/usr/bin/env python3
import time
import serial
import struct
# import pygame
import RPi.GPIO as GPIO
import Encoder
import Odometry
import threading

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
cur_x = 0.0
cur_y = 0.0
cur_w = 0.0
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w, E1, E2, E3, E4
    while True:
        cur_x = Robot.getxDist()
        cur_y = Robot.getyDist()
        cur_w = Robot.getzDist(E1,E2,E3,E4,cur_w)
        E1 = enc1.getDist()
        E2 = enc2.getDist()
        E3 = enc3.getDist()
        E4 = enc4.getDist()
        print(cur_w)
        time.sleep(0.01)

def SendData(Vx,Vy,Wz):
#     valList = [str(Vx),str(Vy),str(Wz)]
#     sendStr = ','.join(valList)
#     print(sendStr)
#     ser.write(sendStr.encode('utf-8'))
#     line = ser.readline().decode('utf-8').rstrip()
    data = struct.pack('fff', Vx, Vy, Wz)
    ser.write(data)
    #print(data)
   

def MoveRobot(type, dist, speed):
    global cur_x, cur_y, cur_w
    Vx = 0
    Vy = 0
    Wz = 0

    # Check if distance is positive. Else set speed to negative
    dir = -1 if (dist<0) else 1
    speed *= dir
#     targetDist = curDist + dist
    if type == 0:
            Vx = speed
            targetDist = cur_x + dist
            
    elif type == 1:
            Vy = speed
            targetDist = cur_y + dist
    elif type == 2:
            Wz = speed
            targetDist = cur_w + dist
    
    if dir > 0:
        while (cur_x <= targetDist) if type == 0 else (cur_y <= targetDist) if type == 1 else(cur_w <= targetDist):
            
            SendData(Vx,Vy,Wz)
    elif dir < 0:
        while (cur_x >= targetDist) if type == 0 else (cur_y >= targetDist) if type == 1 else(cur_w >= targetDist):
            
            SendData(Vx,Vy,Wz)
    
    SendData(0,0,0)
    
        


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
                MoveRobot(2,3.4,1.5)
                end_flag=True
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
            
            time.sleep(0.01)
#             window_surface.blit(background, (0, 0))
    except KeyboardInterrupt:
        SendData(0,0,0)
        print("Exiting...")
        
    finally:
        GPIO.cleanup()