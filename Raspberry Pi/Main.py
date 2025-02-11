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

# Constants
RAD_S_TO_PWM = 255/(0.10472*200)
WHEEL_RADIUS = 0.03
LX, LY = 0.068, 0.061
DEBUG_1,DEBUG_2,DEBUG_3,DEBUG_4,DEBUG_5 = 0.0,0.0,0.0,0.0,0.0
# HI,OK,RECIEVED,MOTOR,ODOM,CLEAR,ERROR = 100,200,500,700,600,650,404
# ATTEMPTS = 3
# Map
start_position = (16,16)
# Variables & Objects
enc1 = Encoder.Encoder(4,5,20)
enc2 = Encoder.Encoder(6,12,20)
enc3 = Encoder.Encoder(13,16,20)
enc4 = Encoder.Encoder(17,18,20)
robot = Odometry.Mecanum_Drive(enc1,enc2,enc3,enc4)
robot_IMU = IMU.Mpu()
end_flag = False
init_yaw = False
whole_path = []
# is_connected = False
# Encoders
enc1_val, enc2_val, enc3_val, enc4_val = 0.0, 0.0, 0.0, 0.0
# Velocities
v1, v2, v3, v4 = 0.0, 0.0, 0.0, 0.0
# Robot Odometry
cur_x = 0.0
cur_y = 0.0
cur_w = 0.0 # Z distance
theta = 0.0 # heading
starting_angle = 0.0 # 
# Fused Odometry
mouse_x, mouse_y = 0.0, 0.0
# Arduino Data
arduino_data = [0.0,0.0,0.0]
# Robot Position
robot_x, robot_y = 0.0, 0.0
r_x,r_y = 0.0,0.0
prev_x,prev_y = 0.0,0.0
# PID
pidx = PID.PID(15,0.2,10)
pidy = PID.PID(15,0.2,10)
pidw = PID.PID(20,0.2,0)
pidw1 = PID.PID(0.2,0.01,0.0)
pidw2 = PID.PID(0.2,0.01,0.0)
pidw3 = PID.PID(0.2,0.01,0.0)
pidw4 = PID.PID(0.2,0.01,0.0)
timeint = 0
# DataLogging
CoordData = []
timelog = []
# Functions
logger = logging.getLogger(__name__)
plot_handler = logging.FileHandler('plot.log')
plot_handler.setLevel(logging.INFO)
plot_format = logging.Formatter('%(asctime)s:%(message)s')
plot_handler.setFormatter(plot_format)
logger.addHandler(plot_handler)
# logging.basicConfig(filename="MainLog.log",format='%(asctime)s %(levelname)s:%(name)s:%(message)s', level=logging.INFO)
def updateOdometry():
    global cur_x, cur_y, cur_w, theta, enc1_val, enc2_val, enc3_val, enc4_val, v1, v2, v3, v4, arduino_data, init_yaw, timeint, robot_x, robot_y, mouse_x, mouse_y
    prev_time = time.time()
    elapsed = 0.0
    while True:
        cur_time = time.time()

        # Serial communication
        # while not is_connected:
        #     print("Waiting for arduino...")
        #     time.sleep(0.1)

        ReadData()
   
        if abs(arduino_data[0]) > 180: # if imu gives gibberish values
            logging.error("IMU error! Restart code")
            print("IMU error! Restart code")
            event.set()
            break

        if (timeint>10):
            if arduino_data[0] == 0: # if imu not transmitting
                logging.error("Serial connection error! Restart Arduino")
                print("Serial connection error! Restart Arduino")
                event.set()
                break
            if not init_yaw:
                robot_IMU.setInitialYaw(arduino_data[0]) # calculate imu offset 
                logging.info(f"INITIAL YAW CALIBRATED: {arduino_data[0]}")
            init_yaw = True
            
        
        # IMU
        omega, theta = robot_IMU.getOdometry(arduino_data)
        # Odometry
        # mouse_x += arduino_data[1]
        # mouse_y += arduino_data[2]
        fused_coord = robot.getFusedOdometry((arduino_data[1],arduino_data[2]),(robot.getxDist(),robot.getyDist()))
        cur_x = fused_coord[0]+start_position[0]
        cur_y = fused_coord[1]+start_position[1]
        cur_w = omega
        # enc1_val = enc1.getDist()
        # enc2_val = enc2.getDist()
        # enc3_val = enc3.getDist()
        # enc4_val = enc4.getDist()
        v1 = enc1.getVel(cur_time)
        v2 = enc2.getVel(cur_time)
        v3 = enc3.getVel(cur_time)
        v4 = enc4.getVel(cur_time)
        robot_x, robot_y = UpdatePosition(cur_x,cur_y,theta)

        if not init_yaw:   
            timeint += 1
        else:
            timeint = 0

        if event.is_set(): # Exit thread
            break

        # Debugging
        if (cur_time - prev_time > 2):
            elapsed += (cur_time - prev_time)
            CollectData((cur_x), cur_y)
            logger.info((robot_x, robot_y))
            prev_time = cur_time

        # print(f"{robot_x} : {robot_y} : {cur_x} :{cur_y} : {cur_w} : {theta}")                                                             
        # print(f"ENCODER VALUES-> {enc1_val}:{enc2_val}:{enc3_val}:{enc4_val}")
        # print(f"VELOCITY VALUES-> {v1}:{v2}:{v3}:{v4}")

        time.sleep(0.05)

def UpdatePosition(x,y,theta)->tuple:
    global prev_x,prev_y,r_x,r_y

    rad = math.radians
    cos = math.cos
    sin = math.sin
    t = rad(theta)
    
    delta_x = x - prev_x
    delta_y = y - prev_y
    delta_worldx = cos(t)*delta_x + sin(t)*delta_y
    delta_worldy = -sin(t)*delta_x + cos(t)*delta_y
    
    r_x += delta_worldx
    r_y += delta_worldy
    
    prev_x = x
    prev_y = y


    return round(r_x,2),round(r_y,2)

def CollectData(data,time):
    CoordData.append(data)
    timelog.append(time)

def ReadData():
    global arduino_data
    # if ser.in_waiting>=2:
    #     msg = ser.read(2)
    #     order = struct.unpack('h',msg)
    #     if (order[0] == HI):
    #         is_connected = True
    #         print("success")
    #         ser.write(struct.pack('h', OK))
    #     elif (order[0] == OK):
    #         print("ok")
    #     elif (order[0] == RECIEVED):
    #         print("recievd")
    #         return True
    #     elif (order[0] == 600):
    #         print("imu")
    #         ser_data = ser.read(12)
    #         arduino_data = struct.unpack('fff',ser_data)
    #     elif (order[0] == 404):
    #         print("ERRoR")
    if ser.in_waiting >=12:
        ser_data = ser.read(12)
        arduino_data = struct.unpack('fff',ser_data)

def SendData(w1,w2,w3,w4):
    # try:
    #     if order == MOTOR:
    #         ser.write(struct.pack('h', order))
    #         ser.write(struct.pack('ffff', package))
    #     elif order == CLEAR:
    #         ser.write(struct.pack('h', order))
    #         ser.write(struct.pack('h', package))
    #     #print(data)
    # except serial.SerialException as e:
    #     logging.error(f"Error writing data to serial port: {e} |in SendData()|")
    #     print("serial error")
    # except Exception as e:
    #     logging.error(f"An unexpected error occurred: {e} |in SendData()|")
    try:
        data = struct.pack('ffff', w1, w2, w3, w4)
        ser.write(data)
        #print(data)
    except serial.SerialException as e:
        logging.error(f"Error writing data to serial port: {e} |in SendData()|")
        print("serial error")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e} |in SendData()|")

def map_values(num, inMin, inMax, outMin, outMax)->float:
    try:
        val = outMin + (float(num - inMin) / float(inMax - inMin) * float(outMax - outMin))
    except ZeroDivisionError as e:
        logging.error(f"Error: {e} |in map_values()|")
    return val

def PID_Controller(x,y,w):
    global DEBUG_1,DEBUG_2,DEBUG_3,DEBUG_4,DEBUG_5
    Vx = 0.0
    Vy = 0.0
    Wz = 0.0
    end_flag = False 
    w_diff = w-cur_w
    while not end_flag:
        PID_time = time.time()
        x_val,endx = pidx.Calculate(x,cur_x,PID_time)
        y_val,endy = pidy.Calculate(y,cur_y,PID_time)
        w_val,endw = pidw.Calculate(w,cur_w,PID_time,0.02,math.pi/2,0.02)
        x_limit = 100
        y_limit = 100
        w_limit = 40 if w_diff>0.2 else 10
        x_speedlim = 0.2
        y_speedlim = 0.2 
        w_speedlim = 1.5
        
        if x_val != 0:
            Vx = map_values(x_val,-x_limit,x_limit,-x_speedlim,x_speedlim)
            Vx = max(min(Vx, x_speedlim), -x_speedlim)
        if y_val != 0:
            Vy = map_values(y_val,-y_limit,y_limit,-y_speedlim,y_speedlim)
            Vy = max(min(Vy, y_speedlim), -y_speedlim)
        if w_val != 0:
            Wz = map_values(w_val,-w_limit,w_limit,-w_speedlim,w_speedlim)
            Wz = max(min(Wz, w_speedlim), -w_speedlim)
            
        if endx and endy and endw:
            end_flag = True
        
        wheel_speed1 = 1/WHEEL_RADIUS * (Vy+Vx + (LX+LY)*Wz)
        wheel_speed2 = 1/WHEEL_RADIUS * (Vy-Vx - (LX+LY)*Wz)
        wheel_speed3 = 1/WHEEL_RADIUS * (Vy-Vx + (LX+LY)*Wz)
        wheel_speed4 = 1/WHEEL_RADIUS * (Vy+Vx - (LX+LY)*Wz)
        
        w1_val, endw1 = pidw1.Calculate(wheel_speed1,v1,PID_time,1,15,1)
        w2_val, endw2 = pidw2.Calculate(wheel_speed2,v2,PID_time,1,15,1)
        w3_val, endw3 = pidw3.Calculate(wheel_speed3,v3,PID_time,1,15,1)
        w4_val, endw4 = pidw4.Calculate(wheel_speed4,v4,PID_time,1,15,1)

        pwm1, pwm2, pwm3, pwm4 = wheel_speed1*RAD_S_TO_PWM, wheel_speed2*RAD_S_TO_PWM, wheel_speed3*RAD_S_TO_PWM, wheel_speed4*RAD_S_TO_PWM

        w1 = (pwm1 + w1_val)
        w2 = (pwm2 + w2_val)
        w3 = (pwm3 + w3_val)
        w4 = (pwm4 + w4_val)
        
        w1 = max(min(w1, 255), -255)
        w2 = max(min(w2, 255), -255)
        w3 = max(min(w3, 255), -255)
        w4 = max(min(w4, 255), -255)
        
        DEBUG_1,DEBUG_2,DEBUG_3,DEBUG_4,DEBUG_5 = w_val, w1, w1_val, pwm1, w1
        SendData(w1,w2,w3,w4)
#         logging.info("{} {} {} {}".format(w1,w1_val,pwm1,W1))
#         logging.debug("{} {} {}".format(i1,i2,i3))
#         logging.debug("{} {} {}".format(endx,endy,endw))
#         logging.debug("{} {} {}".format(x,y,w))
#         logging.debug("{} {} {}".format(Vx,Vy,Wz))
#         logging.debug(f"Coord: {cur_x} {cur_y}")
#         logging.debug("limits:{} {} {}".format(round(x_limit,2),round(y_limit,2),round(w_limit,2)))
#         logging.debug("val:{} {} {}".format(round(x_val,2),round(y_val,2),round(w_val,2)))
#         
        
        
#         time.sleep(0.05)
    logging.info("==========================[ PID Completed ]==========================")
    return end_flag

        
def MoveRobot(type, dist):
    logging.info("==========================[ MoveRobot() Started ]==========================")
    setpoint = dist
    
    x = cur_x
    y = cur_y
    w = cur_w
    
    if type == 0:
        x = setpoint+cur_x
        
    elif type == 1:
        y = setpoint+cur_y
        
    elif type == 2:
        w = setpoint+cur_w
        
    PID_Controller(x,y,w)

    SendData(0,0,0,0)
#     ser.flush()
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    logging.info("==========================[ MoveRobot() Ended ]==========================")
       
def RotateToAngle(angle):
    ''' Rotates robot to a specified heading'''
    logging.info("==========================[ RotateToAngle() Started ]==========================")
   
    
    x = cur_x
    y = cur_y
    sign = math.copysign(1,(angle - theta))
    
    while (theta>angle-0.5 and theta<angle+0.5)==False:
        
        PID_Controller(x,y,math.pi*2*sign)

    SendData(0,0,0,0)

    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    logging.info("==========================[ RotateToAngle() Ended ]==========================")
    
def MoveToCoord(target_x, target_y,init_w = 0):
    logging.info("==========================[ MoveToCoord() Started ]==========================")
    angle = math.radians(theta)
    x = 0
    y = 0
    w = init_w if init_w != 0 else cur_w # when using Astar, to remember the theta value at the start
    x =  math.cos(angle)*target_x - math.sin(angle)*target_y
    y =  math.sin(angle)*target_x + math.cos(angle)*target_y
    x = x-start_position[0] if x<0 else x
    y = y-start_position[1] if y<0 else y
    
    PID_Controller(x,y,w)
    SendData(0,0,0,0)
    
#     ser.flush()
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()
    logging.info("==========================[ MoveToCoord() Completed ]==========================")

def Move_Astar(target_x,target_y):
    global data, path, whole_path
    logging.info("==========================[ Move_Astar() Started ]==========================")
    start = (robot_x,robot_y)
    path, data = Astar.main(start,(target_x,target_y))
    init_w = copy.copy(cur_w)
    try:
        while not end_flag:
            for x,y in path:
                MoveToCoord(x,y,init_w)
                time.sleep(0.1)
            
            break
        for coords in data:
            whole_path.append(coords)
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
                MoveRobot(2,2*math.pi)
                MoveToCoord(60,60)
#                 time.sleep(2)
#                 Move_Astar(160,90)

                end_flag=True
            logging.info("==========================[ Task Ended ]==========================")
            time.sleep(0.1)

    except KeyboardInterrupt:
        SendData(0,0,0,0)
        SendData(500,0,0,0)
        ser.flush()
        event.set()
        update_odometry_thread.join()
        print("Exiting...")
        #         plt.plot(timelog,CoordData)
#         plt.axis((0, 60, -60, 100))
#         print(timelog)
#         print(CoordData)
#         plt.show()
#         plt.plot(whole_path[0][0],whole_path[0][1])
#         plt.plot(16,16,'bo')
# #         plt.plot(whole_path[len(path)-1][0],path[len(path)-1][1],'go')
#         plt.plot(whole_path[1][0],whole_path[1][1],'ks')
#         plt.plot(whole_path[2][0],whole_path[2][1],'rx')
#         RobotPathX = []
#         RobotPathY = []

#         for x,y in CoordData:
#             RobotPathX.append(x)
#             RobotPathY.append(y)
        
#         plt.plot(RobotPathX,RobotPathY,'y--')
#         plt.axis((0, 180, 0, 120))
#         plt.show()
        logging.info("==========================[ Program Ended ]==========================")
        
    finally:
        GPIO.cleanup()


