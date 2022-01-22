
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from numpy import genfromtxt
import matplotlib.pyplot as plt
import subprocess as sub
from tkinter.ttk import Progressbar
import tkinter as tk
import numpy as np
import threading 
import csv
import os
import time


plt.style.use('fivethirtyeight')

root = tk.Tk()
root.geometry("1200x950")
continuePlotting = False



var_source =tk.IntVar()
var_source.set(1)
var_plot = tk.IntVar()
var_plot.set(1)
#################### this in needed for plotting ############################
# plot2, ax2 = plt.subplots(1,2)
plot2, ax2 = plt.subplots(1,3)
# ax2[0].set_title("height of the drone")
# ax2[1].set_title("z velocity of the drone")
ax2[0].set_title("x of the maker")
ax2[1].set_title("y of the maker")
ax2[2].set_title("yaw of the marker")
canvas = FigureCanvasTkAgg(plot2, master=root)  # A tk.DrawingArea.

plot1 = plt.figure()
ax1= plot1.add_subplot(projection='3d')
# canvas = FigureCanvasTkAgg(plot1, master=root)  # A tk.DrawingArea.
canvas.draw()
canvas.get_tk_widget().pack(fill=tk.X)
##############################################################################
def change_state():
    global continuePlotting
    if continuePlotting == True:
        continuePlotting = False
    else:
        continuePlotting = True


def plotter():
    global continuePlotting
    global height_val
    while continuePlotting:
        # try:
            # data = genfromtxt('test_data.csv', delimiter=',')
            # print(height_val)

            if var_plot.get() == 1:
                data = genfromtxt('goal_pos.csv', delimiter=',')                   
                height_val_vec = height_val*np.ones(len(data)) 
                ax2[0].clear()
                ax2[0].plot(data[2:,0])

                ax2[0].set_title("x of the maker")

                ax2[1].clear()
                ax2[1].plot(data[2:,1])
                ax2[1].set_title("y of the maker")

                ax2[2].clear()
                ax2[2].plot(data[2:,2])
                ax2[2].plot(height_val_vec/100, 'r--', linewidth=1)
                ax2[2].set_title("yaw of the marker")
            # canvas = FigureCanvasTkAgg(plot2, master=root)  # A tk.DrawingArea.

            if var_plot.get() == 2:         
                data = genfromtxt('velocity_drone.csv', delimiter=',') 
                data_real = genfromtxt('velocity_drone_real.csv', delimiter=',')    
                height_val_vec = height_val*np.ones(len(data)) 
                ax2[0].clear()
                ax2[0].plot(data[2:,0]) ## this send 
                ax2[0].plot(data_real[2:,0], 'r') ## this is the actual velocity
                ax2[0].plot(height_val_vec/100, 'r--', linewidth=1)
                ax2[0].set_title("velocity in x")

                ax2[1].clear()
                ax2[1].plot(data[2:,1])
                ax2[1].plot(data_real[2:,1], "r") ## this is the actual velocity
                ax2[1].set_title("velocity in y")

                ax2[2].clear()
                ax2[2].plot(data[2:,2])
                # ax2[2].plot(data_real[2:,2], "r") ## this is the actual velocity
                ax2[2].set_title("rotational velocity")

            # my_data = genfromtxt('pos_marker.csv', delimiter=',')
            # ax1= plot1.add_subplot(projection='3d')
            # ax1.plot3D(my_data[:,0], my_data[:,1], my_data[:,2], 'gray')
            # canvas = FigureCanvasTkAgg(plot1, master=root)  # A tk.DrawingArea.

            # plt.tight_layout()
            canvas.draw()
        # except FileNotFoundError:
        #     print("no csv file present")
        #     continuePlotting = False

def gui_handler():
    global thread
    change_state()
    thread = threading.Thread(target=plotter).start()  

#################### this in needed for plotting ############################

def pub_height():
    global height_val
    height_val =  int(scl_height.get())
    print(height_val)
    height_label_val.set(str(height_val))
 
    # sub.call(['rostopic', 'pub', '-1', '/custom_command', 'std_msgs/String', "z"+str(height_val)])
    sub.call(['rostopic', 'pub', '-1', '/custom_command', 'std_msgs/Float32', str(height_val)])


def pub_speed():
    global speed_val
    speed_val =  float(scl_speed.get()/100.0)
    print(speed_val)
    speed_label_val.set(str(speed_val))

    sub.call(['rostopic', 'pub', '-1', '/custom_command', 'std_msgs/String', "s"+str(speed_val)])

def clean_up():
    global thread
    try:
        thread.test()
        thread.join()
    except:
        pass
    root.quit()
    root.destroy()

def clear_plots():
    try:
        os.remove("goal_pos.csv")
    except:
        pass

    with open("goal_pos.csv", "w",  newline='') as my_empty_csv:
        my_empty_csv_w = csv.writer(my_empty_csv)
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        pass  

    try:
        os.remove("velocity_drone.csv")
    except:
        pass

    with open("velocity_drone.csv", "w",  newline='') as my_empty_csv:
        my_empty_csv_w = csv.writer(my_empty_csv)
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        pass 

    try:
        os.remove("velocity_drone_real.csv")
    except:
        pass

    with open("velocity_drone_real.csv", "w",  newline='') as my_empty_csv:
        my_empty_csv_w = csv.writer(my_empty_csv)
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        my_empty_csv_w.writerow([0.0 , 0.0 ,0.0])
        pass          
    # ax1.clear()
    

def sel():
    print("print"+str(var_plot.get()))
    if var_plot.get() == 1:
        plot2, ax2 = plt.subplots(1,3)
        ax2[0].set_title("x of the maker")
        ax2[1].set_title("y of the maker")
        ax2[2].set_title("z of the maker")
        canvas = FigureCanvasTkAgg(plot2, master=root)  # A tk.DrawingArea.

    if var_plot.get() == 2:
        # plot1 = plt.figure()
        # ax1= plot1.add_subplot(projection='3d')
        # canvas = FigureCanvasTkAgg(plot1, master=root)  # A tk.DrawingArea.
        plot2, ax2 = plt.subplots(1,3)
        ax2[0].set_title("velocity in x")
        ax2[1].set_title("velocity in z")
        ax2[2].set_title("rotational velocity")
        canvas = FigureCanvasTkAgg(plot2, master=root)  # A tk.DrawingArea.
    canvas.draw()
    # canvas.get_tk_widget().pack(fill=tk.X)

def select_source():
    print("print "+str(var_source.get()))




# tk.Label(root, text="Drone operation app!").pack() # Create a text label

frm_tool = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=3)
frm_tool.pack(pady=5)

frm_main_run = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=3)
frm_main_run.pack(pady=5)

frm_drone_controll = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=3)
frm_drone_controll.pack(pady=5)


frm_state_and_slider = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=3)
frm_state_and_slider.pack(pady=5)

frm_slider = tk.Frame(master=frm_state_and_slider, relief=tk.RIDGE, borderwidth=3)
frm_slider.pack(side=tk.LEFT)

frm_state = tk.Frame(master=frm_state_and_slider)
frm_state.pack(side=tk.LEFT)

tk.Label(master=frm_slider, text="target height in cm").pack() # Create a text label
frm_height = tk.Frame(master=frm_slider, relief=tk.RIDGE, borderwidth=3)
frm_height.pack(pady=5)

tk.Label(master=frm_slider, text="speed control ").pack() # Create a text label
frm_speed = tk.Frame(master=frm_slider, relief=tk.RIDGE, borderwidth=3)
frm_speed.pack(pady=5)

tk.Label(master=root, text="plot functions").pack() # Create a text label
frm_plot = tk.Frame(master=root, relief=tk.RIDGE, borderwidth=3)
frm_plot.pack(pady=5)

def run_script():
    # print("hallo")
    if var_source.get() == 1:
        sub.call(['gnome-terminal', '-x','python', '/home/valentin/catkin_ws/src/competition_v1/src/fly_drone.py'])
    if var_source.get() == 2:
        sub.call(['gnome-terminal', '-x','python', '/home/valentin/catkin_ws/src/competition_v1/src/fly_simulation.py'])

def marker_detection():
    # print("hallo")
    if var_source.get() == 1:
        # print("hich bin nummer 1")
        sub.call(['gnome-terminal', '-e', 'roslaunch ar_track_alvar my.launch'])
    if var_source.get() == 2:
        # print("hich bin nummer 2")
        sub.call(['gnome-terminal', '-e', 'roslaunch ar_track_alvar my_simulation.launch'])

def marker_detection_competition():
        sub.call(['gnome-terminal', '-e', 'roslaunch ar_track_alvar my_competition.launch'])



tk.Button(master=frm_tool, text="roscore", command=lambda: sub.call(['gnome-terminal', '-e', 'roscore'])).pack(side=tk.LEFT)
# tk.Button(master=frm_tool, text="connect drone", command=lambda: sub.call(['gnome-terminal', '-e', 'roslaunch bebop_tools bebop_nodelet_iv.launch'])).pack(side=tk.LEFT)
tk.Button(master=frm_tool, text="show image", command=lambda: sub.call(['gnome-terminal', '-e', 'rosrun image_view image_view image:=/bebop/image_raw'])).pack(side=tk.LEFT)
# tk.Button(master=frm_tool, text="run marker detection", command=lambda: sub.call(['gnome-terminal', '-e', 'roslaunch ar_track_alvar my.launch'])).pack(side=tk.LEFT)
tk.Button(master=frm_tool, text="run ros bag", command=lambda: sub.call(['gnome-terminal', '-e', 'rosbag play /home/valentin/drone_base/long.bag'])).pack(side=tk.LEFT)
tk.Button(master=frm_tool, text="start Gazebo simulation", command=lambda: sub.call(['gnome-terminal', '-e', 'roslaunch sjtu_drone ardrone_simulation.launch'])).pack(side=tk.LEFT)

tk.Button(master=frm_main_run, text="connect drone", command=lambda: sub.call(['gnome-terminal', '-e', 'roslaunch bebop_driver bebop_node.launch'])).pack(side=tk.LEFT)
tk.Button(master=frm_main_run, text="run marker detection\ntest", command=marker_detection).pack(side=tk.LEFT)
tk.Button(master=frm_main_run, text="run marker detection\ncompetition ", command=marker_detection_competition).pack(side=tk.LEFT)
# tk.Button(master=frm_main_run, text="run marker detection\ncompetition ", command=marker_detection_competition).pack(side=tk.LEFT)

tk.Button(master=frm_main_run, text="run main script", command=run_script, bg="green", fg="white").pack(side=tk.LEFT)


frm_source_choice = tk.Frame(master=frm_tool, relief=tk.RIDGE, borderwidth=3)
frm_source_choice.pack(side=tk.LEFT)
R1_source = tk.Radiobutton(master=frm_source_choice, text="Real drone", variable=var_source, value=1,command=select_source).pack(anchor = tk.W)
R2_source = tk.Radiobutton(master=frm_source_choice, text="Simulation", variable=var_source, value=2,command=select_source).pack(anchor = tk.W)

def drone_land():
    # print("hallo")
    if var_source.get() == 1:
        # print("hich bin nummer 1")
        sub.call(['rostopic', 'pub', '-1', 'bebop/land', 'std_msgs/Empty'])
    if var_source.get() == 2:
        # print("hich bin nummer 2")
        sub.call(['rostopic', 'pub', '-1', 'drone/land', 'std_msgs/Empty'])


def drone_take_off():
    print("hallo")
    if var_source.get() == 1:
        # print("hich bin nummer 1")
        sub.call(['rostopic', 'pub', '-1', 'bebop/takeoff', 'std_msgs/Empty'])
    if var_source.get() == 2:
        # print("hich bin nummer 2")
        sub.call(['rostopic', 'pub', '-1', 'drone/takeoff', 'std_msgs/Empty'])



def update_state():
    ## read from svc
    while True:
        time.sleep(0.5)
        current_state = genfromtxt("current_state.csv")
        if current_state == 0: # state Take off
            state_canvas_follow.itemconfig(my_oval0, fill="green")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2,  fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")
     
        state_canvas_follow.itemconfig(my_oval3, fill="")
        if current_state == 1: # Looking for marker 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="green")
            state_canvas_follow.itemconfig(my_oval2, fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")

        if current_state == 2:  # following maker 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="green")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")
        if current_state == 3:  # following maker 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="green")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")
        if current_state == 4:   # landing 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="green")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")
        if current_state == 5:   #  
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="green")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="")

        if current_state == 8:   # landing 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="green")
            state_canvas_land.itemconfig(my_oval8, fill="")

        if current_state == 8:   # landing 
            state_canvas_follow.itemconfig(my_oval0, fill="")
            state_canvas_follow.itemconfig(my_oval1, fill="")
            state_canvas_follow.itemconfig(my_oval2, fill="")
            state_canvas_follow.itemconfig(my_oval3, fill="")

            state_canvas_land.itemconfig(my_oval4, fill="")
            state_canvas_land.itemconfig(my_oval5, fill="")
            state_canvas_land.itemconfig(my_oval7, fill="")
            state_canvas_land.itemconfig(my_oval8, fill="red")
        ## update cancas
        root.update()

# tk.Button(master=frm_drone_controll, text="Drone Land",  command=lambda: sub.call(['rostopic', 'pub', '-1', 'bebop/land', 'std_msgs/Empty']), bg="red", fg="white").pack(side=tk.LEFT)
tk.Button(master=frm_drone_controll, text="Drone Land",  command=drone_land, bg="red", fg="white").pack(side=tk.LEFT)
tk.Button(master=frm_drone_controll, text="stop drone", command=lambda: sub.call(['rostopic', 'pub', '-1', '/custom_command','std_msgs/String', 'stop'])).pack(side=tk.LEFT)

# tk.Button(master=frm_drone_controll, text="rotate camera \n forwad", command=lambda: sub.call(['rostopic pub /bebop/camera_control geometry_msgs/Twist "linear:\ x: 0.0\ y: 0.0\ z: 0.0\angular:\ x: 0.0\ y: 0.0\ z: 0.0"'])).pack(side=tk.LEFT)
tk.Button(master=frm_drone_controll, text="rotate camera \n down", command=lambda: sub.call(['gnome-terminal', '-e', 'rostopic pub -1 /bebop/camera_control geometry_msgs/Twist -f /home/valentin/camera_rotation_down.yaml'])).pack(side=tk.LEFT)
tk.Button(master=frm_drone_controll, text="rotate camera \n forward", command=lambda: sub.call(['gnome-terminal', '-e', 'rostopic pub -1 /bebop/camera_control geometry_msgs/Twist -f /home/valentin/camera_rotation_forward.yaml'])).pack(side=tk.LEFT)

# rostopic pub /bebop/camera_control geometry_msgs/Twist -f rotation_forwar.yaml
tk.Button(master=frm_drone_controll, text="Drone Take off",  command=drone_take_off, bg="green", fg="white").pack(side=tk.LEFT)


### plotting
tk.Button(master=frm_plot, text="show plots", command=gui_handler).pack(side=tk.LEFT)
tk.Button(master=frm_plot, text="clear plots", command=clear_plots).pack(side=tk.LEFT)

frm_plot_choice = tk.Frame(master=frm_plot, relief=tk.RIDGE, borderwidth=3)
frm_plot_choice.pack(side=tk.LEFT)
R1 = tk.Radiobutton(master=frm_plot_choice, text="goal pose", variable=var_plot, value=1,command=sel).pack(anchor = tk.W)
R2 = tk.Radiobutton(master=frm_plot_choice, text="velocities", variable=var_plot, value=2,command=sel).pack(anchor = tk.W)


height_val = 0
height_label_val = tk.StringVar()
height_label_val.set(str(height_val))
lab_height = tk.Label(master=frm_height,textvariable=height_label_val).pack(side=tk.LEFT)
scl_height = tk.Scale(master=frm_height, from_=0, to=200,length=300,  orient=tk.HORIZONTAL)
scl_height.set(height_val)
scl_height.pack(side=tk.LEFT)
tk.Button(master=frm_height, text="pub height", command=pub_height).pack(side=tk.LEFT)

speed_val = 1.0
speed_label_val = tk.StringVar()
speed_label_val.set(str(speed_val))
lab_speed = tk.Label(master=frm_speed,textvariable=speed_label_val).pack(side=tk.LEFT)
scl_speed = tk.Scale(master=frm_speed, from_=0, to=400,length=300, orient=tk.HORIZONTAL)
scl_speed.set(speed_val)
scl_speed.pack(side=tk.LEFT)
tk.Button(master=frm_speed, text="pub speed", command=pub_speed).pack(side=tk.LEFT)



state_canvas_follow = tk.Canvas(master = frm_state, width=165, height=120)
state_canvas_follow.pack(side=tk.LEFT)
state_canvas_land = tk.Canvas(master = frm_state, width=165, height=120)
state_canvas_land.pack(side=tk.LEFT)

# follow
my_oval0 = state_canvas_follow.create_oval(1, 1, 30, 30)  # Create a circle on the Canvas
my_oval1 = state_canvas_follow.create_oval(1, 30, 30, 60)  # Create a circle on the Canvas
my_oval2 = state_canvas_follow.create_oval(1, 60, 30, 90)  # Create a circle on the Canvas
my_oval3 = state_canvas_follow.create_oval(1, 90, 30, 120)  # Create a circle on the Canvas
lab_state_0 = tk.Label(master=frm_state, text="Take off").place(x=35,y=0)
lab_state_1 = tk.Label(master=frm_state, text="Looking for marker").place(x=35,y=30)
lab_state_2 = tk.Label(master=frm_state, text="Following marker").place(x=35,y=60)
lab_state_3 = tk.Label(master=frm_state, text="moving over vehicle").place(x=35,y=90)

# landing
my_oval4 = state_canvas_land.create_oval(1, 1, 30, 30)  # Create a circle on the Canvas
my_oval5 = state_canvas_land.create_oval(1, 30, 30, 60)  # Create a circle on the Canvas
my_oval7 = state_canvas_land.create_oval(1, 60, 30, 90)  # Create a circle on the Canvas
my_oval8 = state_canvas_land.create_oval(1, 90, 30, 120)  # Create a circle on the Canvas
lab_state_4 = tk.Label(master=frm_state, text="rotaion camera").place(x=200,y=0)
lab_state_5 = tk.Label(master=frm_state, text="position over marker").place(x=200,y=30)
lab_state_7 = tk.Label(master=frm_state, text="find missing maker").place(x=200,y=60)
lab_state_8 = tk.Label(master=frm_state, text="Landing").place(x=200,y=90)
# lab_state_3 = tk.Label(master=frm_state, text="Landing").place(x=35,y=90)


uppdate_thread = threading.Thread(target=update_state)
uppdate_thread.start()


tk.Button(root, text="exit", command=clean_up, bg="red", fg="white").pack()


##################################  Battery indicator ###########################
# def battery_update():
#     root.update_idletasks()
#     battery = genfromtxt('battery.csv', delimiter=',') 
#     print(battery)
#     pb_battery['value'] = battery
#     time.sleep(1)
#     # txt_battery['text'] = str(actual_battery[-1]),'%'
# pb_battery = Progressbar(root, length = 100,mode = 'determinate').pack()
# txt_battery = tk.Label(root, text = '0%').pack()
# bat = tk.Button(root,text="hallo" ,command=battery_update).pack()
##################################  Battery indicator ###########################

root.mainloop()