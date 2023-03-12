#!/usr/bin/env python3
import time
import serial
import RPi.GPIO as GPIO
import Encoder

# Define GPIO pins

# Initialize GPIO

# Variables
enc1 = Encoder.Encoder(17,18)
# Functions

if __name__ == '__main__':

#     try:
#         ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
#     except:
#         ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
#     ser.reset_input_buffer()
    try:
        while True:
            ## PUT CODE HERE ##
            print(enc1.getCount())
#         
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
