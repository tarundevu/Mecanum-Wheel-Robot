'''
This file contains the main code for the control of a mecanum robot
             
       ______                          ______          __  
      /_  __/___ ________  ______     /_  __/__  _____/ /_ 
       / / / __ `/ ___/ / / / __ \     / / / _ \/ ___/ __ \         
      / / / /_/ / /  / /_/ / / / /    / / /  __/ /__/ / / /
     /_/  \__,_/_/   \__,_/_/ /_/    /_/  \___/\___/_/ /_/ 

                                                      
2023
'''
import time
import serial
import struct
import matplotlib.pyplot as plt
import math
import copy
import RPi.GPIO as GPIO
import Encoder
import Odometry
import threading
import PID
import IMU
import Astar
import logging

# Map
StartPos = (16,16)
# Variables & Objects
enc1 = Encoder.Encoder(4,5,20)
enc2 = Encoder.Encoder(6,12,20)
enc3 = Encoder.Encoder(13,16,20)
enc4 = Encoder.Encoder(17,18,20)
Robot = Odometry.Mecanum_Drive(enc1,enc2,enc3,enc4)
Robot_IMU = IMU.Mpu()
end_flag = False
init_yaw = False
# Encoders
E1, E2, E3, E4 = 0.0, 0.0, 0.0, 0.0
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
# Fused Odometry
# IMU Data
IMU_Val = [0.0]
# Robot Position
Robot_x = 0.0
Robot_y = 0.0
# PID
pidx = PID.PID(10,0.1,10)
pidy = PID.PID(10,0.1,10)
pidw = PID.PID(2,0.01,0)
pidw1 = PID.PID(0.5,0.0001,0.5)
pidw2 = PID.PID(0.5,0.0001,0)
pidw3 = PID.PID(0.5,0.0001,0)
pidw4 = PID.PID(0.5,0.0001,0)
timeint = 0
# DataLogging
CoordData = []
timelog = []
# Functions
logging.basicConfig(filename="MainLogs.log",format='%(asctime)s %(levelname)s:%(name)s:%(message)s', level=logging.INFO)
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4, V1, V2, V3, V4, IMU_Val, init_yaw, timeint, Robot_x, Robot_y
    prev_time = time.time()
    elapsed = 0.0
    while True:
        cur_time = time.time()

        # Serial communication
        if ser.in_waiting >=4:
            imudata = ser.read(4)
            IMU_Val = struct.unpack('f',imudata)
   
            if abs(IMU_Val[0]) > 180: # if imu gives gibberish values
                logging.error("IMU error! Restart code")
                event.set()
                break

        if (cur_time - prev_time > 3):
            if IMU_Val[0] == 0: # if imu not transmitting
                logging.error("Serial connection error! Restart Arduino")
                event.set()
                break
            if not init_yaw:
                Robot_IMU.setInitialYaw(IMU_Val[0]) # calculate imu offset 
                logging.info(f"INITIAL YAW CALIBRATED: {IMU_Val[0]}")
            init_yaw = True
            
        
        # IMU
        w_total, theta = Robot_IMU.getOdometry(IMU_Val)
        # Odometry
        cur_x = Robot.getxDist() + StartPos[0]
        cur_y = Robot.getyDist() + StartPos[1]
        cur_w = w_total
        # E1 = enc1.getDist()
        # E2 = enc2.getDist()
        # E3 = enc3.getDist()
        # E4 = enc4.getDist()
        V1 = enc1.getVel(cur_time)
        V2 = enc2.getVel(cur_time)
        V3 = enc3.getVel(cur_time)
        V4 = enc4.getVel(cur_time)
        Robot_x, Robot_y = UpdatePosition(cur_x,cur_y,theta)

        if not init_yaw:   
            timeint += 1
        else:
            timeint = 0

        if event.is_set(): # Exit thread
            break

        # Debugging
        if (cur_time - prev_time > 2):
            elapsed += (cur_time - prev_time)
            CollectData((cur_y),elapsed)
            prev_time = cur_time

        # print(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {theta}")
        # logging.debug(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {theta}")                                                               
        # logging.debug(f"ENCODER VALUES-> {E1}:{E2}:{E3}:{E4}")
        # logging.debug(f"VELOCITY VALUES-> {V1}:{V2}:{V3}:{V4}")

        time.sleep(0.05)

def UpdatePosition(x,y,theta)->tuple:
    rad = math.radians
    cos = math.cos
    sin = math.sin
    t = rad(theta)
    r_x = cos(t)*x + sin(t)*y
    r_y = -sin(t)*x + cos(t)*y
    
    return round(r_x,2),round(r_y,2)

def CollectData(data,time):
    CoordData.append(data)
    timelog.append(time)

def SendData(w1,w2,w3,w4):
    try:
        data = struct.pack('ffff', w1, w2, w3, w4)
        ser.write(data)
        #print(data)
    except serial.SerialException as e:
        logging.error(f"Error writing data to serial port: {e} |in SendData()|")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e} |in SendData()|")

def map_values(num, inMin, inMax, outMin, outMax)->float:
    try:
        val = outMin + (float(num - inMin) / float(inMax - inMin) * float(outMax - outMin))
    except ZeroDivisionError as e:
        logging.error(f"Error: {e} |in map_values()|")
    return val

def PID_Controller(x,y,w):

    Vx = 0.0
    Vy = 0.0
    Wz = 0.0
    end_Flag = False 
    x_diff = x - cur_x
    y_diff = y - cur_y
    w_diff = w - cur_w
    wheel_radius = 0.03
    lx,ly = 0.068, 0.061
    rad_s_to_pwm = 255/(0.10472*200)
    while not end_Flag:
        PID_time = time.time()
        x_val,endx = pidx.Calculate(x,cur_x,PID_time,0.2)
        y_val,endy = pidy.Calculate(y,cur_y,PID_time,0.2)
        w_val,endw = pidw.Calculate(w,cur_w,PID_time,0.02,0.02)
        x_limit = abs(x_diff+0.001)*20
        y_limit = abs(y_diff+0.001)*20
        w_limit = abs(w_diff+0.001)*10

        
        x_speed_lim = 0.1 if abs(x_diff)<30 else 0.3
        y_speed_lim = 0.1 if abs(y_diff)<30 else 0.3 
        w_speed_lim = 1.5 if abs(w)<=math.pi/2 else 2.5
        
        if x_val != 0:
            Vx = map_values(x_val,-x_limit,x_limit,-x_speed_lim,x_speed_lim)
            Vx = max(min(Vx, x_speed_lim), -x_speed_lim)
        if y_val != 0:
            Vy = map_values(y_val,-y_limit,y_limit,-y_speed_lim,y_speed_lim)
            Vy = max(min(Vy, y_speed_lim), -y_speed_lim)
        if w_val != 0:
            Wz = map_values(w_val,-w_limit,w_limit,-w_speed_lim,w_speed_lim)
            Wz = max(min(Wz, w_speed_lim), -w_speed_lim)
            
        
        w1 = 1/wheel_radius * (Vy + Vx +(lx + ly)*Wz)
        w2 = 1/wheel_radius * (Vy - Vx -(lx + ly)*Wz)
        w3 = 1/wheel_radius * (Vy - Vx +(lx + ly)*Wz)
        w4 = 1/wheel_radius * (Vy + Vx -(lx + ly)*Wz)
        
        w1_val, endw1 = pidw1.Calculate(w1,V1,PID_time,0.01,0.01,1)
        w2_val, endw2 = pidw2.Calculate(w2,V2,PID_time,0.01,0.01,1)
        w3_val, endw3 = pidw3.Calculate(w3,V3,PID_time,0.01,0.01,1)
        w4_val, endw4 = pidw4.Calculate(w4,V4,PID_time,0.01,0.01,1)

        pwm1, pwm2, pwm3, pwm4 = w1*rad_s_to_pwm, w2*rad_s_to_pwm, w3*rad_s_to_pwm, w4*rad_s_to_pwm

        W1 = (pwm1 + w1_val)
        W2 = (pwm2 + w2_val)
        W3 = (pwm3 + w3_val)
        W4 = (pwm4 + w4_val)
        
        W1 = max(min(W1, 255), -255)
        W2 = max(min(W2, 255), -255)
        W3 = max(min(W3, 255), -255)
        W4 = max(min(W4, 255), -255)
    
        SendData(W1,W2,W3,W4)
#         logging.debug("{} {} {} {}".format(W1,W2,W3,W4))
#         logging.debug("{} {} {}".format(i1,i2,i3))
#         logging.debug("{} {} {}".format(endx,endy,endw))
#         logging.debug("{} {} {}".format(x,y,w))
#         logging.debug("{} {} {}".format(Vx,Vy,Wz))
#         logging.debug(f"Coord: {cur_x} {cur_y}")
#         logging.debug("limits:{} {} {}".format(round(x_limit,2),round(y_limit,2),round(w_limit,2)))
#         logging.debug("val:{} {} {}".format(round(x_val,2),round(y_val,2),round(w_val,2)))
#         
        if endx and endy and endw:
            end_Flag = True
        
#         time.sleep(0.05)
    logging.info("==========================[ PID Completed ]==========================")
    return end_Flag

        
def MoveRobot(type, dist):
    logging.info("==========================[ MoveRobot() Started ]==========================")
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
        
    PID_Controller(x,y,w)

    SendData(0,0,0,0)
#     ser.flush()
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    logging.info("==========================[ MoveRobot() Ended ]==========================")
       
        
def MoveToCoord(target_x, target_y,init_w = 0):
    logging.info("==========================[ MoveToCoord() Started ]==========================")
    angle = math.radians(theta)
    x = 0
    y = 0
    w = init_w if init_w != 0 else cur_w # when using Astar, to remember the theta value at the start
    x =  math.cos(angle)*target_x - math.sin(angle)*target_y
    y =  math.sin(angle)*target_x + math.cos(angle)*target_y
    
    PID_Controller(x,y,w)
    SendData(0,0,0,0)
    
#     ser.flush()
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    logging.info("==========================[ MoveToCoord() Completed ]==========================")

def Move_Astar(target_x,target_y):
    global data, path
    logging.info("==========================[ Move_Astar() Started ]==========================")
    start = (Robot_x,Robot_y)
    path, data = Astar.main(start,(target_x,target_y))
    init_w = copy.copy(cur_w)
    try:
        while not end_flag:
            for x,y in path:
                MoveToCoord(x,y,init_w)
                time.sleep(0.1)
            
            break
    except:
        logging.warning("No path found!")
    logging.info("==========================[ Move_Astar() Completed ]==========================")
        
if __name__ == '__main__':
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    ser.reset_input_buffer()
    ser.flush()
    update_odometry_thread = threading.Thread(target=updateOdometry)
    event = threading.Event()
    update_odometry_thread.daemon = True
    update_odometry_thread.start()
    logging.info("==========================[ Program Started ]==========================")
    time.sleep(3)
    try:
        while True:
            if not end_flag:

                Move_Astar(75,100)
                time.sleep(2)
                Move_Astar(160,90)

                end_flag=True
            logging.info("==========================[ Task Ended ]==========================")
            time.sleep(0.1)

            # PID plot
            
            
            # Robot path plot
#             plt.plot(data[0][0],data[0][1])
#             plt.plot(16,16,'bo')
#             plt.plot(path[len(path)-1][0],path[len(path)-1][1],'go')
#             plt.plot(data[1][0],data[1][1],'ks')
#             plt.plot(data[2][0],data[2][1],'rx')
#             RobotPathX = []
#             RobotPathY = []
# 
#             for x,y in CoordData:
#                 RobotPathX.append(x)
#                 RobotPathY.append(y)
#             
#             plt.plot(RobotPathX,RobotPathY,'y--')
#             plt.axis((0, 180, 0, 120))

    except KeyboardInterrupt:
        SendData(0,0,0,0)
        ser.flush()
        event.set()
        update_odometry_thread.join()
        print("Exiting...")
        plt.plot(timelog,CoordData)
        plt.axis((0, 60, -60, 100))
        print(timelog)
        print(CoordData)
        logging.info("==========================[ Program Ended ]==========================")
        
    finally:
        GPIO.cleanup()
