
import threading
import time
import PID
import math
import tkinter as tk
import Astar
import IMU
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import logging
# Map
StartPos = (16,16)
# Variables & Objects
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
V_x = 0.0
V_y = 0.0
W_z = 0.0
# Robot Odometry
Robot_IMU = IMU.Mpu()
cur_x = 0.0 +  StartPos[0]
cur_y = 0.0 +  StartPos[1]
cur_w = 0.0 # Z distance
theta = 0.0 # heading
# Fused Odometry
# Robot Position
Robot_x = 0.0
Robot_y = 0.0
# PID
pidx = PID.PID(10,0.01,8)
pidy = PID.PID(10,0.01,8)
pidw = PID.PID(15,0.01,5)
pidw1 = PID.PID(2,0.0001,0.5)
pidw2 = PID.PID(2,0.0001,0.5)
pidw3 = PID.PID(2,0.0001,0.5)
pidw4 = PID.PID(5,0.0001,0.5)
timeint = 0
#debug variables
Pwm1 = 0.0
Pwm2 = 0.0
Pwm3 = 0.0
Pwm4 = 0.0
newvalx = 0.0
newvaly = 0.0
newvalw = 0.0
# GUI
g_x,g_y = np.array([StartPos[0]]),np.array([StartPos[1]])
# DataLogging
CoordData = []
timelog = []
# Functions

def updateOdometry():
    global cur_w, Robot_x,Robot_y,g_x,g_y
    prev_time = time.time()
    elapsed = 0.0
    while True:
        cur_time = time.time()
        cur_w,th = Robot_IMU.getOdometry([theta])
        Robot_x, Robot_y = UpdatePosition(cur_x,cur_y,th)
        g_x = np.append(g_x,Robot_x)
        g_y = np.append(g_y,Robot_y)

        # print(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {th}")
        # print(f"{V_x},{V_y},{W_z}")

        # Debugging
        if (cur_time - prev_time > 2):
            elapsed += (cur_time - prev_time)
            CollectData((cur_x,cur_y),elapsed)
            prev_time = cur_time
        time.sleep(0.05)

def CollectData(data,time):
    CoordData.append(data)
    timelog.append(time)

def GUI_start():

    def addCury():
        global cur_y
        cur_y += 1
        update()
    def subCury():
        global cur_y
        cur_y -= 1
        update()
    def addCurx():
        global cur_x
        cur_x += 1
        update()
    def subCurx():
        global cur_x
        cur_x -= 1
        update()
    def addCurw():
        global theta
        theta += 1
        update()
    def subCurw():
        global theta
        theta -= 1
        update()
    def update():
        label_x.config(text=round(cur_x,2))
        label_y.config(text=round(cur_y,2))
        label_w.config(text=round(cur_w,2))
        label_RX.config(text=round(Robot_x,2))
        label_RY.config(text=round(Robot_y,2))
        label_theta.config(text=round(theta,2))
        label_nx.config(text=round(newvalx,2))
        label_ny.config(text=round(newvaly,2))
        label_nw.config(text=round(newvalw,2))
        label_pwm1.config(text=round(Pwm1,2))
        label_pwm2.config(text=round(Pwm2,2))
        label_pwm3.config(text=round(Pwm3,2))
        label_pwm4.config(text=round(Pwm4,2))
        label_vx.config(text=round(V_x,2))
        label_vy.config(text=round(V_y,2))
        label_wz.config(text=round(W_z,2))
  
    window = tk.Tk()
    window.title('ROBOT DROID TEST CONSOLE')
    bg = tk.Canvas(window, width=400,height=330 )
    icon_small = tk.PhotoImage(file='C:/Users/Loan Unit/Documents/Mecanum-Wheel-Robot-main/Mecanum-Wheel-Robot-main/Raspberry Pi/HUD_16x16.png')
    icon_large = tk.PhotoImage(file='C:/Users/Loan Unit/Documents/Mecanum-Wheel-Robot-main/Mecanum-Wheel-Robot-main/Raspberry Pi/HUD_1_32x32.png')
    window.iconphoto(False,icon_large,icon_small)
    window.resizable(False,False)
    curYbutton_up = tk.Button(window,bg="light grey",fg="black",text='\u2303',height=1,width=1,command=addCury)
    curYbutton_down = tk.Button(window,bg="light grey",fg="black",text='\u2304',height=1,width=1,command=subCury)
    curXbutton_left = tk.Button(window,bg="light grey",fg="black",text='<',height=1,width=1,command=subCurx)
    curXbutton_right = tk.Button(window,bg="light grey",fg="black",text='>',height=1,width=1,command=addCurx)
    curWbutton_cw = tk.Button(window,bg="light grey",fg="black",text='\u21B7',height=1,width=1,command=addCurw)
    curWbutton_ccw = tk.Button(window,bg="light grey",fg="black",text='\u21B6',height=1,width=1,command=subCurw)
    curYbutton_up.place(x=250, y=200)
    curYbutton_down.place(x=250, y=240)
    curXbutton_left.place(x=230, y=220)
    curXbutton_right.place(x=270, y=220)
    curWbutton_cw.place(x=270, y=180)
    curWbutton_ccw.place(x=230, y=180)
    label_x = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(cur_x,2))
    label_y = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(cur_y,2))
    label_w = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(cur_w,2))
    label_RY = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Robot_x,2))
    label_RX = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Robot_y,2))
    label_theta = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(theta,2))
    label_nx = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(newvalx,2))
    label_ny = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(newvaly,2))
    label_nw = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(newvalw,2))
    label_pwm1 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Pwm1,2))
    label_pwm2 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Pwm2,2))
    label_pwm3 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Pwm3,2))
    label_pwm4 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(Pwm4,2))
    label_vx = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(V_x,2))
    label_vy = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(V_y,2))
    label_wz = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(W_z,2))
    x = tk.Label(borderwidth=1,anchor="center",text="cur x")
    y = tk.Label(borderwidth=1,anchor="center",text="cur y")
    w = tk.Label(borderwidth=1,anchor="center",text="cur w")
    robot_x = tk.Label(borderwidth=1,anchor="center",text="robot x")
    robot_y = tk.Label(borderwidth=1,anchor="center",text="robot y")
    robot_th = tk.Label(borderwidth=1,anchor="center",text="theta")
    new_x = tk.Label(borderwidth=1,anchor="center",text="new x")
    new_y = tk.Label(borderwidth=1,anchor="center",text="new y")
    new_w = tk.Label(borderwidth=1,anchor="center",text="new w")
    pwm_1 = tk.Label(borderwidth=1,anchor="center",text="pwm 1")
    pwm_2 = tk.Label(borderwidth=1,anchor="center",text="pwm 2")
    pwm_3 = tk.Label(borderwidth=1,anchor="center",text="pwm 3")
    pwm_4 = tk.Label(borderwidth=1,anchor="center",text="pwm 4")
    v_x = tk.Label(borderwidth=1,anchor="center",text="Vx")
    v_y = tk.Label(borderwidth=1,anchor="center",text="Vy")
    w_z = tk.Label(borderwidth=1,anchor="center",text="Wz")
    x.place(x=30,y=50)
    y.place(x=30,y=75)
    w.place(x=30,y=100)
    robot_x.place(x=90,y=50)
    robot_y.place(x=90,y=75)
    robot_th.place(x=90,y=100)
    new_x.place(x=160,y=50)
    new_y.place(x=160,y=75)
    new_w.place(x=160,y=100)
    pwm_1.place(x=235,y=50)
    pwm_2.place(x=235,y=75)
    pwm_3.place(x=235,y=100)
    pwm_4.place(x=235,y=125)
    v_x.place(x=10,y=170)
    v_y.place(x=70,y=170)
    w_z.place(x=140,y=170)
    label_x.place(x=66,y=50)
    label_y.place(x=66,y=75)
    label_w.place(x=66,y=100)
    label_RX.place(x=132,y=50)
    label_RY.place(x=132,y=75)
    label_theta.place(x=132,y=100)
    label_nx.place(x=200,y=50)
    label_ny.place(x=200,y=75)
    label_nw.place(x=200,y=100)
    label_pwm1.place(x=280,y=50)
    label_pwm2.place(x=280,y=75)
    label_pwm3.place(x=280,y=100)
    label_pwm4.place(x=280,y=125)
    label_vx.place(x=30,y=170)
    label_vy.place(x=90,y=170)
    label_wz.place(x=160,y=170)

    bg.pack()
    window.mainloop()
    
    

def UpdatePosition(x,y,theta):
    rad = math.radians
    cos = math.cos
    sin = math.sin
    t = rad(theta)
    r_x = cos(t)*x + sin(t)*y
    r_y = -sin(t)*x + cos(t)*y
    
    return round(r_x,2),round(r_y,2)

def plot_map():
    return (Robot_x,Robot_y)
        
        
    
def map_values(num, inMin, inMax, outMin, outMax)->float:
    return (num - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def PID_Controller(x,y,w):
    global Pwm1,Pwm2,Pwm3,Pwm4,newvalx,newvaly,newvalw,V_x,V_y,W_z
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
    multiplier = w*(15+10+5)
    while not end_Flag:
        PID_time = time.time()
        x_val,endx = pidx.Calculate(x,cur_x,PID_time,0.2)
        y_val,endy = pidy.Calculate(y,cur_y,PID_time,0.2)
        w_val,endw = pidw.Calculate(w,cur_w,PID_time,0.02,0.05,10,True)
        x_limit = abs(x_diff+0.001)*20
        y_limit = abs(y_diff+0.001)*20
        w_limit = 0.75*multiplier if w_diff>0.16 else 0.5
        x_speed_lim = 0.1 if abs(x_diff)<30 else 0.25
        y_speed_lim = 0.1 if abs(y_diff)<30 else 0.25 
        w_speed_lim = 1.5 
        
        newvalx,newvaly,newvalw = x_val,y_val,w_val
        if x_val != 0:
            Vx = map_values(x_val,-x_limit,x_limit,-x_speed_lim,x_speed_lim)
            Vx = max(min(Vx, x_speed_lim), -x_speed_lim)
        if y_val != 0:
            Vy = map_values(y_val,-y_limit,y_limit,-y_speed_lim,y_speed_lim)
            Vy = max(min(Vy, y_speed_lim), -y_speed_lim)
        if w_val != 0:
            Wz = map_values(w_val,-w_limit,w_limit,-w_speed_lim,w_speed_lim)
            Wz = max(min(Wz, w_speed_lim), -w_speed_lim)
            
        V_x,V_y,W_z = Vx,Vy,Wz

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

        Pwm1,Pwm2,Pwm3,Pwm4 = W1,W2,W3,W4
    
#         logging.debug("{} {} {} {}".format(W1,W2,W3,W4))
#         logging.debug("{} {} {}".format(i1,i2,i3))
#         logging.debug("{} {} {}".format(endx,endy,endw))
#         logging.debug("{} {} {}".format(x,y,w))
        # print("{} {} {}".format(Vx,Vy,Wz))
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
    # global cur_x, cur_y, cur_w
    
    setpoint = dist
    x,y,w = 0.0, 0.0, 0.0
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

    print("*******************MoveRobot Seq Complete*********************")
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()

def MoveToCoord(target_x, target_y):
    angle = math.radians(theta)
    x = 0
    y = 0
    w = cur_w
    x =  math.cos(angle)*target_x - math.sin(angle)*target_y
    y =  math.sin(angle)*target_x + math.cos(angle)*target_y
    
    PID_Controller(x,y,w)
    print("*********************MoveToCoord Seq complete********************")
    pidx.Reset()
    pidy.Reset()
    pidw.Reset()

def Move_Astar(target_x,target_y): # add total path
    global data, path
    path, data = Astar.main(StartPos,(target_x,target_y))
    
    while not end_flag:
        for x,y in path:
            MoveToCoord(x,y)
            print(x,y)
        
        break

if __name__ == '__main__':
    update_odometry_thread = threading.Thread(target=updateOdometry)
    GUI_thread = threading.Thread(target=GUI_start)
    update_odometry_thread.daemon = True
    GUI_thread.daemon = True
    update_odometry_thread.start()
    GUI_thread.start()
    time.sleep(3)
    try:
        
        while not end_flag:
       
            # Move_Astar(25,40)

            MoveRobot(2,math.pi*2)

            # rr = input("e")
            # if rr == 'e':
            end_flag=True
            
            time.sleep(0.5)
            
        # PID plot
        # plt.plot(timelog,CoordData)
        # plt.axis((0, 60, -60, 100))
        
        # Robot path plot
        # plt.plot(data[0][0],data[0][1])
        # plt.plot(16,16,'bo')
        # plt.plot(path[len(path)-1][0],path[len(path)-1][1],'go')
        # plt.plot(data[1][0],data[1][1],'ks')
        # plt.plot(data[2][0],data[2][1],'rx')
        # RobotPathX = []
        # RobotPathY = []

        # for x,y in CoordData:
        #     RobotPathX.append(x)
        #     RobotPathY.append(y)
        
        # plt.plot(RobotPathX,RobotPathY,'y--')
        # plt.axis((0, 180, 0, 120))
 

        plt.show()
    except KeyboardInterrupt:
        print("Exiting...")
      
