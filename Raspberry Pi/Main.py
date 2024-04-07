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
IMU_Val = [0.0]
# Robot Position
Robot_x = 0.0
Robot_y = 0.0
# PID
pidx = PID.PID(10,0.1,8)
pidy = PID.PID(10,0.1,8)
pidw = PID.PID(2,0.01,7)
pidw1 = PID.PID(2,0.001,0)
pidw2 = PID.PID(2,0.001,0)
pidw3 = PID.PID(2,0.001,0)
pidw4 = PID.PID(2,0.001,0)
timeint = 0
# DataLogging
CoordData = []
# Functions
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4, V1, V2, V3, V4, IMU_Val, init_yaw, timeint, Robot_x,Robot_y
    prev_time = time.time()
    while True:
        cur_time = time.time()

        # Serial communication
        if ser.in_waiting >=4:
            imudata = ser.read(4)
            IMU_Val = struct.unpack('f',imudata)
   
            if abs(IMU_Val[0]) > 180:
                print("--------------IMU ERROR!------------")
                print("--------------Please Reset------------")
                event.set()
                break

        if timeint>10:
            if IMU_Val[0] == 0:
                print("---------------MPU 6050 ERROR!--------------")
                break
            if not init_yaw:
                Robot_IMU.setInitialYaw(IMU_Val[0])
                print("Yaw Calibrated")
                print(IMU_Val[0])
            init_yaw = True
            
        
        # IMU
        pose, theta = Robot_IMU.getOdometry(IMU_Val)
        # Odometry
        cur_x = Robot.getxDist() + StartPos[0]
        cur_y = Robot.getyDist() + StartPos[1]
        cur_w = pose 
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

        if event.is_set():
            break

        # Debugging
        if (cur_time - prev_time > 2):
            CollectData(cur_x)
            prev_time = cur_time

        print(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {theta}")
        # print("The time of execution of above program is :",(end_time-cur_time) * 10**3, "ms")                                                                       
        # print("{}:{}:{}::{}".format(E1,E2,E3,E4))
        # print(f"{V1}:{V2}:{V3}:{V4}")

        time.sleep(0.05)

def UpdatePosition(x,y,theta):
    rad = math.radians
    cos = math.cos
    sin = math.sin
    t = rad(theta)
    r_x = cos(t)*x + sin(t)*y
    r_y = -sin(t)*x + cos(t)*y
    
    return round(r_x,2),round(r_y,2)

def CollectData(data):
    CoordData.append(data)
def SendData(w1,w2,w3,w4):
    try:
        data = struct.pack('ffff', w1, w2, w3, w4)
        ser.write(data)
        #print(data)
    except serial.SerialException as e:
        print(f"Error writing data to serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def map_values(num, inMin, inMax, outMin, outMax):
    try:
        val = outMin + (float(num - inMin) / float(inMax - inMin) * float(outMax - outMin))
    except ZeroDivisionError as e:
        print(f"Error: {e}")
    return val

def PID_Controller(x,y,w):

    Vx = 0
    Vy = 0
    Wz = 0
    end_Flag = False 
    x_diff = x - cur_x
    y_diff = y - cur_y
    w_diff = w - cur_w
    wheel_radius = 0.03
    lx,ly = 0.068, 0.061
    while not end_Flag:
        x_val,i1,endx = pidx.Calculate(x,cur_x,0.02)
        y_val,i2,endy = pidy.Calculate(y,cur_y,0.02)
        w_val,i3,endw = pidw.Calculate(w,cur_w,0.02,0.02)
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
            Wz = map_values(w_val,-w_limit,w_limit,-w_spd_lim,w_spd_lim)
            Wz = max(min(Wz, w_spd_lim), -w_spd_lim)
            
#         print("{} {} {}".format(i1,i2,i3))
#         print("{} {} {}".format(endx,endy,endw))
#         print("{} {} {}".format(x,y,w))
#         print("{} {} {}".format(Vx,Vy,Wz))
#         print(f"Coord: {cur_x} {cur_y}")
#         print("limits:{} {} {}".format(round(x_limit,2),round(y_limit,2),round(w_limit,2)))
#         print("val:{} {} {}".format(round(x_val,2),round(y_val,2),round(w_val,2)))
        
        w1 = 1/wheel_radius * (Vy + Vx +(lx + ly)*Wz)
        w2 = 1/wheel_radius * (Vy - Vx -(lx + ly)*Wz)
        w3 = 1/wheel_radius * (Vy - Vx +(lx + ly)*Wz)
        w4 = 1/wheel_radius * (Vy + Vx -(lx + ly)*Wz)
        
        w1_val,_1,g = pidw1.Calculate(w1,V1,0.01,0.01)
        w2_val,_11,g1 = pidw2.Calculate(w2,V2,0.01,0.01)
        w3_val,_12,g2 = pidw3.Calculate(w3,V3,0.01,0.01)
        w4_val,_13,g3 = pidw4.Calculate(w4,V4,0.01,0.01)

        W1 = (w1/0.10472 + w1_val)
        W2 = (w2/0.10472 + w2_val)
        W3 = (w3/0.10472 + w3_val)
        W4 = (w4/0.10472 + w4_val)
        
        W1 = max(min(W1, 255), -255)
        W2 = max(min(W2, 255), -255)
        W3 = max(min(W3, 255), -255)
        W4 = max(min(W4, 255), -255)
    
        SendData(W1,W2,W3,W4)
#         print(W1)
        if endx and endy and endw:
            end_Flag = True
        
#         time.sleep(0.05)
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
        
    PID_Controller(x,y,w)

    SendData(0,0,0,0)
#     ser.flush()
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
    print("*********************MoveToCoord Seq complete********************")
    SendData(0,0,0,0)
#     ser.flush()
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
                time.sleep(0.1)
            
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
    ser.flush()
    update_odometry_thread = threading.Thread(target=updateOdometry)
    event = threading.Event()
    update_odometry_thread.daemon = True
    update_odometry_thread.start()
    time.sleep(3)
    try:
        while True:
            if not end_flag:
    #               MoveRobot(2,2*math.pi,0.2)
                MoveRobot(1,90)
#                 time.sleep(3)
#                 SendData(0,0,0)
#                 MoveToCoord(60,60)
#                 MoveToCoord(72,44)
#                 MoveToCoord(126,44)
#                 MoveToCoord(140,60)
#                 SendData(30,0,0)
#                 time.sleep(4)
# #             if not end_flag:
#                 MoveRobot(0,2) 
#                 Move_Astar(75,100)
#                 time.sleep(2)
#                 Move_Astar(160,90

                end_flag=True

            time.sleep(0.1)

            plt.plot(CoordData[0],CoordData[1])
            plt.axis((0, 100, -100, 100))
            plt.show()

    except KeyboardInterrupt:
        SendData(0,0,0,0)
        ser.flush()
        event.set()
        update_odometry_thread.join()
        print("Exiting...")
        
    finally:
        GPIO.cleanup()
