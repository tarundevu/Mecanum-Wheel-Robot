import threading
import time
import PID
import math
import tkinter as tk
import Astar
import IMU

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
pidx = PID.PID(8,0.5,10)
pidy = PID.PID(10,0.5,10)
pidw = PID.PID(10,0.01,8)
timeint = 0
#debug variables
int1 = 0.0
int2 = 0.0
int3 = 0.0
newvalx = 0.0
newvaly = 0.0
newvalw = 0.0
# GUI
# Functions

def updateOdometry():
    # global cur_x, cur_y, cur_w, theta, E1, E2, E3, E4,V1,V2,V3,V4, IMU_Val, init_yaw, timeint, Robot_x,Robot_y
    global cur_w, Robot_x,Robot_y
    while True:
        cur_w,th = Robot_IMU.getOdometry([0,0,theta])
        Robot_x, Robot_y = UpdatePosition(cur_x,cur_y,th)
        
        print(f"{Robot_x} : {Robot_y} : {cur_x} :{cur_y} : {cur_w} : {th}")
        
        time.sleep(0.05)
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
        label_int1.config(text=round(int1,2))
        label_int2.config(text=round(int2,2))
        label_int3.config(text=round(int3,2))
    window = tk.Tk()
    window.title('ROBOT DROID TEST CONSOLE')
    bg = tk.Canvas(window, width=400,height=330 )
    icon_small = tk.PhotoImage(file='C:/github/Mecanum-Wheel-Robot/Raspberry Pi/rover.png')
    icon_large = tk.PhotoImage(file='C:/github/Mecanum-Wheel-Robot/Raspberry Pi/rover_large.png')
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
    label_int1 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(int1,2))
    label_int2 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(int2,2))
    label_int3 = tk.Label(window,background="white",borderwidth=1,anchor="center",text=round(int3,2))
    x = tk.Label(borderwidth=1,anchor="center",text="cur x")
    y = tk.Label(borderwidth=1,anchor="center",text="cur y")
    w = tk.Label(borderwidth=1,anchor="center",text="cur w")
    robot_x = tk.Label(borderwidth=1,anchor="center",text="robot x")
    robot_y = tk.Label(borderwidth=1,anchor="center",text="robot y")
    robot_th = tk.Label(borderwidth=1,anchor="center",text="theta")
    new_x = tk.Label(borderwidth=1,anchor="center",text="new x")
    new_y = tk.Label(borderwidth=1,anchor="center",text="new y")
    new_w = tk.Label(borderwidth=1,anchor="center",text="new w")
    int_1 = tk.Label(borderwidth=1,anchor="center",text="int 1")
    int_2 = tk.Label(borderwidth=1,anchor="center",text="int 2")
    int_3 = tk.Label(borderwidth=1,anchor="center",text="int 3")
    x.place(x=30,y=50)
    y.place(x=30,y=75)
    w.place(x=30,y=100)
    robot_x.place(x=90,y=50)
    robot_y.place(x=90,y=75)
    robot_th.place(x=90,y=100)
    new_x.place(x=160,y=50)
    new_y.place(x=160,y=75)
    new_w.place(x=160,y=100)
    int_1.place(x=235,y=50)
    int_2.place(x=235,y=75)
    int_3.place(x=235,y=100)
    label_x.place(x=66,y=50)
    label_y.place(x=66,y=75)
    label_w.place(x=66,y=100)
    label_RX.place(x=132,y=50)
    label_RY.place(x=132,y=75)
    label_theta.place(x=132,y=100)
    label_nx.place(x=200,y=50)
    label_ny.place(x=200,y=75)
    label_nw.place(x=200,y=100)
    label_int1.place(x=266,y=50)
    label_int2.place(x=266,y=75)
    label_int3.place(x=266,y=100)

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

def map_values(num, inMin, inMax, outMin, outMax)->float:
    return (num - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def PID_Controller(x,y,w):
    global int1,int2,int3,newvalx,newvaly,newvalw
    Vx = 0.0
    Vy = 0.0
    Wz = 0.0
    
    end_Flag = False
    init_x,init_y,init_w = x-cur_x,y-cur_y,w-cur_w
    while not end_Flag:
        x_val,i1,endx = pidx.Calculate(x,cur_x,(x+4)*0.75)
        y_val,i2,endy= pidy.Calculate(y,cur_y,(y+4)*0.75)
        w_val,i3,endw = pidw.Calculate(w,cur_w,math.pi/4,0.02,0.02)
        int1,int2,int3 = i1,i2,i3
        newvalx,newvaly,newvalw = x_val,y_val,w_val
        x_limit,y_limit,w_limit = abs(init_x+0.001)*20, abs(init_y+0.001)*20, abs(w_val)*10
        spd_limitx = 0.1 if init_x < 30 else 0.25
        spd_limity = 0.1 if init_y < 30 else 0.25
        spd_limitw = 0.5 if init_w < math.pi/2 else 2
        if x_val != 0:
            Vx = map_values(x_val,-x_limit,x_limit,-spd_limitx,spd_limitx)
            Vx = max(min(Vx, spd_limitx), -spd_limitx)
        if y_val != 0:
            Vy = map_values(y_val,-y_limit,y_limit,-spd_limity,spd_limity)
            Vy = max(min(Vy, spd_limity), -spd_limity)
        if w_val != 0:
            Wz = map_values(w_val,-w_limit,w_limit,-spd_limitw,spd_limitw)
            Wz = max(min(Wz, spd_limitw), -spd_limitw)
        
#     print("{}".format(endx))
    # print("working")
        
        # print("{} {} {}".format(y,y_val,w_val))
    # print("{} {} {}".format(x_limit,y_limit,w_limit))
    # print("{} {} {}".format(Vx,Vy,Wz))
    
        if endx and endy and endw:
            end_Flag = True
            print("******************PID seq complete****************")
    # time.sleep(0.5)
    # print("{} {} {}".format(endx,endy,endw))

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
#     global theta, end_flag
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

def Move_Astar(target_x,target_y):
    path = Astar.main(StartPos,(target_x,target_y))
    
    while not end_flag:
        for x,y in path:
            MoveToCoord(x,y)
            print(x,y)
            # path.remove((x,y))
        
        if path==None:
            print("xxxxxxxxxxxxxxxxxxxxxxxxxx_Astar Seq Complete_xxxxxxxxxxxxxxxxxxxxxxxxx")
            break

if __name__ == '__main__':
    update_odometry_thread = threading.Thread(target=updateOdometry)
    GUI_thread = threading.Thread(target=GUI_start)
    update_odometry_thread.daemon = True
    GUI_thread.daemon = True
    update_odometry_thread.start()
    GUI_thread.start()
    #initialize yaw
        # cur_w = 
    
    time.sleep(3)
    try:
        
        while not end_flag:
            # 
            Move_Astar(160,90)
            end_flag=True
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
      