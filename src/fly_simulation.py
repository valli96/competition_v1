#!/usr/bin/env python2.7
from tf.transformations import decompose_matrix, euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Float32, Header
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg  import AlvarMarkers

from tf import TransformListener
# from sensor_msgs.msg import CompressedImage

# import pandas as pd
import csv
import threading
import math
import time
import rospy
import numpy as np
import sys
from simple_pid import PID

# import pandas
from drone_pid import PID_controller 

# global state_of_operation 

state_of_operation = 0 # 0 starting Drone 
                       # 1 looking for the marker 
                       # 2 following the marker
                       # 3 landing on the marker


goal_pos = Point()
theta_marker = 0
Empty_ = Empty()
speed = Twist()

rot_Mat = np.array([[0, 0,  1],
                    [1, 0,  0],
                    [0, -1, 0]])

first_run = True
initial_odom = [0, 0, 0, 0]  # [x , y , z, yaw]
initial_odom_theta = 0


speed_factor = 1
custom_height = 1
custom_x = 0.0
custom_y = 0.0
custom_angel = 0.0



rospy.init_node("speed_controller")
pub_takeoff = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pub_land = rospy.Publisher("/drone/land", Empty, queue_size=1)


def diff_angels(x, y):
    global theta_marker
    x += math.pi
    y += math.pi
    a = (x - y) % (math.pi*2)
    b = (y - x) % (math.pi*2)
    return -a if a < b else b


def dist(x, y, z):
    global goal_pos
    inc_x = goal_pos.x - x
    inc_y = goal_pos.y - y
    inc_z = goal_pos.z - z
    distance = math.sqrt(inc_x**2 + inc_y**2 + inc_z**2)
    return inc_x, inc_y, inc_z, distance


first_run = True
initial_odom_x = 0
initial_odom_y = 0
initial_odom_z = 0
initial_odom_theta = 0

speed_factor = 1
custom_height = 1
custom_x = 0.0

def callback(msg):
    '''
    this function is used for debugging in moving the drone to arbitrary positions
    the first iteration of algorithm don't the the postion of the drone!  

    '''
    global first_run
    global initial_odom
    global custom_x
    global custom_y
    global custom_height
    global custom_angel

    ################## odom message of drone ############################### 
    pos_done = [msg.position.x,
                msg.position.y,
                msg.position.z]

    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    ################## odom message of drone ############################### 
    
    ################################### calibration IMU ######################
    if first_run == True:

        initial_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw]
        first_run = False
    ################################### calibration IMU ######################
    
    ##################### adjust to target pos and pub speed ######################
    pos_done[0] = (msg.pose.pose.position.x - initial_odom[0]) - custom_x # custom_x
    pos_done[1] = (msg.pose.pose.position.y - initial_odom[1]) - custom_y
    pos_done[2] = (msg.pose.pose.position.z - initial_odom[2]) - custom_height  # - 2 the number will define how heigh the drone can hover 
    drone_rot = (msg.pose.pose.position.z - initial_odom[3]) - custom_angel  #  this is the yaw of the drone
    
    pid_x = PID(0.3, 0.05, 0.01, setpoint=0)
    # pid_y = PID(0.3, 0.05, 0.01, setpoint=0)
    pid_z = PID(0.5, 0.01, 0.05, setpoint=0)
    pid_rot = PID(0.5, 0.01, 0.05, setpoint=0)

    speed.linear.x = pid_x(pos_done[0])
    speed.linear.y = 0
    # speed.linear.z = pid_z(pos_done[2])
    speed.linear.z = 0
    speed.angular.z =  pid_rot(drone_rot)
    # pub_move.publish(speed)
    ##################### adjust 
    
    # with open('test_data.csv', 'a') as f:
    #     writer = csv.writer(f)
    #     # writer.writerow([msg.pose.pose.position.z - initial_odom_z, speed.linear.z])
    #     writer.writerow([msg.pose.pose.position.x - initial_odom_x, speed.linear.x])    




def custom_command(msg):
    global custom_height
    custom_height = msg.data/100
    print("The new Target height is !!! :  " + str(custom_height))    
    

def look_for_marker():
    global state_of_operation
    if state_of_operation == 0 or state_of_operation == 2:
        state_of_operation = 1  

def main_algorithm(msg):
    global state_of_operation 
    global custom_x
    global custom_height
    global diff_ang
    print(state_of_operation)

    # if state_of_operation == 2 and ((goal_pos.x and goal_pos.y) < 0.2):
    #     state_of_operation = 3

    if state_of_operation == 1:
        print("Looking for marker...")
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.z = 0
        speed.angular.z = 0.5
        pub_move.publish(speed)

    if state_of_operation == 2:
        print("following the marker")
        pid_x = PID(0.3, 0.05, 0.01, setpoint=0)
        pid_z = PID(0.5, 0.01, 0.05, setpoint=0)
        pid_rot = PID(0.5, 0.01, 0.05, setpoint=0)

        speed.linear.x = pid_x(goal_pos.x)
        speed.linear.y = 0
        speed.linear.z = pid_z(goal_pos.z - 0,5)
        speed.linear.z = 0
        speed.angular.z = pid_rot(diff_ang)
        # pub_move.publish(speed)

    if state_of_operation == 3:
        print("Landing ")
        pub_land.publish(Empty_)

def get_maker_pos_2(msg):
    if msg.markers:
        print("I see the the marker")
        global diff_ang
        global timer_marker
        global state_of_operation
        global goal_pos
        state_of_operation = 1
        # print(msg.markers.pose)

        ######################### position ################################
        vec_pos_mark_m_space = [msg.markers[0].pose.pose.position.x,
                                msg.markers[0].pose.pose.position.y,
                                msg.markers[0].pose.pose.position.z]

        goal_pos.x = vec_pos_mark_m_space[0] #- math.cos(diff_ang)*0.5
        goal_pos.y = -vec_pos_mark_m_space[1] #- math.sin(diff_ang)*0.5
        goal_pos.z = vec_pos_mark_m_space[2]
        print(vec_pos_mark_m_space)
        ######################### position ################################

        ######################### rotation ################################
        (roll, pitch, yaw) = euler_from_quaternion(
            [msg.markers[0].pose.pose.orientation.x, msg.markers[0].pose.pose.orientation.y, msg.markers[0].pose.pose.orientation.z, msg.markers[0].pose.pose.orientation.w])
        theta_marker = roll
        if theta_marker >= 0:
            diff_ang = -(theta_marker - math.pi)
        if theta_marker < 0:
            diff_ang = -(theta_marker + math.pi)
        print(round(theta_marker, 3), "theta angel")
        ######################### rotation ################################
        
        ################ writing to csv File ##############################
        with open('test_data.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow([goal_pos.x, goal_pos.y,goal_pos.z])
            
        # with open('test_data.csv', 'a') as f:
        #     writer = csv.writer(f)
        #     writer.writerow([msg.pose.pose.position.z - initial_odom_z, speed.linear.z])
        ############### writing to csv File ###############################

        ################ for the marker looking routine ###################
        timer_marker.cancel()
        timer_marker = threading.Timer(10, look_for_marker)
        timer_marker.start()    
        ################ for the marker looking routine ###################



###################### if the process is killed ##########################################
def myhook():
    pub_land.publish(Empty_)
    print("shutdown time!")
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    pub_move.publish(speed)
    time.sleep(2)
    print("program aborted drone will land")

rospy.on_shutdown(myhook)
##################### if the process is killed ##########################################

def main():
    global state_of_operation
    global timer_marker
    time.sleep(0.5)
    pub_takeoff.publish(Empty_)
    time.sleep(2.5)
    state_of_operation = 1
    rospy.Subscriber("/drone/gt_pose", Odometry, main_algorithm, queue_size=1)
    # rospy.Subscriber("/visualization_marker", Marker, get_maker_pos, queue_size=1)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_maker_pos_2, queue_size=1)
    # rospy.Subscriber("/bebop/odom", Odometry, callback, queue_size=1)
    rospy.Subscriber("/custom_command", Float32, custom_command, queue_size=1)

    timer_marker = threading.Timer(10,look_for_marker) # If 5 seconds elapse, call look for marker
    timer_marker.start()   
    print("hallo:")
    rospy.spin()

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()