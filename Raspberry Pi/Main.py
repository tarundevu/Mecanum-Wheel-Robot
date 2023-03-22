#!/usr/bin/env python3
import time
import serial
import RPi.GPIO as GPIO
import Encoder
import Odometry

# Define GPIO pins

# Initialize GPIO

# Variables
Robot = Odometry.Mecanum_Drive()

enc1 = Encoder.Encoder(17,18,20)
enc2 = Encoder.Encoder(19,20,20)
enc3 = Encoder.Encoder(21,22,20)
enc4 = Encoder.Encoder(23,24,20)

# Functions
def MoveRobot():
    pass

if __name__ == '__main__':

#     try:
#         ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
#     except:
#         ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
#     ser.reset_input_buffer()
    try:
        while True:
            ## PUT CODE HERE ##
            print(enc1.getDist)
         
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
