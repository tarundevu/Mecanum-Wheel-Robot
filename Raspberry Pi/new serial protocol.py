import struct
import time
from robust_serial import Order, read_i16, read_i8, read_order, write_i8, write_i16, write_order
from robust_serial.utils import open_serial_port

if __name__ == "__main__":
    try:
        serial_file = open_serial_port('COM5',baudrate=115200, timeout=None)
    except Exception as e:
        raise e

    is_connected = False
    # Initialize communication with Arduino
    while not is_connected:
        print("Waiting for arduino...")
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    print("Connected to Arduino")
    
    W1,W2,W3,W4 = 0,0,0,0
    t= 0
    t2= 0
    t3= 0
    t4= 0
    po = []
    stop = False
    while not stop:
        # Equivalent to write_i8(serial_file, Order.MOTOR.value)
        o = input("enter:")
        if o=='y':
            W1 = int(input("enter1:"))
            W2 = int(input("enter2:"))
            W3 = int(input("enter3:"))
            W4 = int(input("enter4:"))
        else:
            stop = True
        
        write_order(serial_file, Order.MOTOR)
        write_i16(serial_file, W1)
        # write_order(serial_file, Order.M2)
        write_i16(serial_file, W2)
        # write_order(serial_file, Order.M3)
        write_i16(serial_file, W3)
        # write_order(serial_file, Order.M4)
        write_i16(serial_file, W4)

        
        order = read_order(serial_file)
        # if (order == Order.IMU):
        t = read_i16(serial_file)
        t1 = read_i16(serial_file)
        t2 = read_i16(serial_file)
        t3 = read_i16(serial_file)
            
        
        print("Ordered received: {:?}", order)
        print(t,t2,t3,t4)
    