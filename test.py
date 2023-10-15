import time 
import math

val = float(input("enter setpoint: "))
yaw = 0
cur_w = 0


while True:
    inp = input("enter integral: ")
    if inp == "y":
        yaw -= 10

    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360
    
    test = yaw
    if test < 0:
        test += 360
    
    change = yaw - val
    if change > 180:
        change -= 360
    elif change < -180:
        change += 360

    cur_w += change

    val = yaw


    print("{} {} {}".format(yaw,cur_w,change))

