#!/usr/bin/env python3
import time
import serial
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

# E1 = 0.0
# E2 = 0.0 
# E3 = 0.0
# E4 = 0.0
cur_x = 0.0
cur_y = 0.0
cur_w = 0.0
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w
    pass
    while True:
        cur_x = Robot.getxDist()
        cur_y = Robot.getyDist()
        cur_w = Robot.getzDist()
        time.sleep(0.01)

def SendData(Vx,Vy,Wz):
    valList = [str(Vx),str(Vy),str(Wz)]
    sendStr = ','.join(valList)
    print(sendStr)
    ser.write(sendStr.encode('utf-8'))
    line = ser.readline().decode('utf-8').rstrip()
    print(line)

def MoveRobot(type, dist, speed):
    global cur_x, cur_y, cur_w
    # Vx = 0
    # Vy = 0
    # Wz = 0
    if type == 0:
        curDist = cur_x
    elif type == 1:
        curDist = cur_y
    targetDist = curDist + dist

    dir = -1 if (dist<0) else 1
    speed *= dir
   
    while (curDist != targetDist+1 or curDist != targetDist-1):
        Vx = 0
        Vy = 0
        Wz = 0
        if type == 0:
            # curDist = Robot.getxDist()
            Vx = speed
        elif type == 1:
            # curDist = Robot.getyDist()
            Vy = speed

        SendData(Vx,Vy,Wz)
        # valList = [str(Vx),str(Vy),str(Wz)]
        # sendStr = ','.join(valList)
        # print(sendStr)
        # ser.write(sendStr.encode('utf-8'))
        # line = ser.readline().decode('utf-8').rstrip()
        # print(line)

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
#             
            MoveRobot(1,100,0.35)         
            
            time.sleep(0.1)
#             window_surface.blit(background, (0, 0))
    except KeyboardInterrupt:
        # update_odometry_thread
        print("Exiting...")
        
    finally:
        GPIO.cleanup()
