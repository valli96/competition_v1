#!/usr/bin/env python2.7
from tf.transformations import decompose_matrix, euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Float32, Header
from visualization_msgs.msg import Marker

from ar_track_alvar_msgs.msg import AlvarMarker

from tf import TransformListener, Transformer
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

state_of_operation = 0 # 0 looking for the marker 
                       # 1 following the marker
                       # 2 landing on the marker
                       # 3 somthing else

goal_pos = Point()
theta_marker = 0
Empty_ = Empty()
speed = Twist()

rospy.init_node("speed_controller")
pub_takeoff = rospy.Publisher("bebop/takeoff", Empty, queue_size=1)
pub_move = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
pub_land = rospy.Publisher("/bebop/land", Empty, queue_size=1)


# def diff_angels(x, y):
#     global theta_marker
#     x += math.pi
#     y += math.pi
#     a = (x - y) % (math.pi*2)
#     b = (y - x) % (math.pi*2)
#     return -a if a < b else b


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
    global first_run
    global initial_odom_x
    global initial_odom_y
    global initial_odom_z
    global initial_odom_theta
    global custom_height
    global custom_x

    ################## odom message of drone ###############################
    pos_done = [msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z]

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    ################## odom message of drone ############################### 
    
    ################################### calibration IMU ######################
    if first_run == True:
        initial_odom_x = msg.pose.pose.position.x
        initial_odom_y = msg.pose.pose.position.y
        initial_odom_z = msg.pose.pose.position.z
        initial_odom_theta = theta
        first_run = False
   ################################### calibration IMU ######################
    pos_done[0] = (msg.pose.pose.position.x - initial_odom_x) - custom_x # custom_x
    pos_done[1] = (msg.pose.pose.position.y - initial_odom_y)
    pos_done[2] = (msg.pose.pose.position.z - initial_odom_z) - custom_height  # - 2 the number will define how heigh the drone can hover 

    pid_x = PID(0.3, 0.05, 0.01, setpoint=0)
    pid_y = PID(0.3, 0.05, 0.01, setpoint=0)
    # pid_z = PID(0.5, 0.01, 0.05, setpoint=0)

    speed.linear.x = pid_x(pos_done[0])
    speed.linear.y = pid_y(pos_done[1])
    # speed.linear.z = pid_z(pos_done[2])
    speed.linear.z = 0
    # pub_move.publish(speed)

def get_maker_pos(msg):
    print("I see the the marker")
    global theta_marker
    global timer_marker
    global state_of_operation
    state_of_operation = 1

    ######################### position ################################
    vec_pos_mark_m_space = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]

    goal_pos.x = vec_pos_mark_m_space[0] #- math.cos(diff_ang)*0.5
    goal_pos.y = -vec_pos_mark_m_space[1] #- math.sin(diff_ang)*0.5
    goal_pos.z = vec_pos_mark_m_space[2]
    # print(vec_pos_mark_m_space)
    ######################### position ################################

    ######################### rotation ################################
    (roll, pitch, yaw) = euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    theta_marker = roll
    if theta_marker >= 0:
        diff_ang = -(theta_marker - math.pi)
    if theta_marker < 0:
        diff_ang = -(theta_marker + math.pi)
    # print(round(theta_marker, 3), "theta angel")
    ######################### rotation ################################



    ######################### transformation ################################
    ###### not needed anymore since the right frame are selected in the ar_marker launch file 
    ## this needs to be changed to the wright transformation 
    # vec_pos_marker_g_space = np.dot((rot_Mat), vec_pos_mark_m_space)
    ######################### transformation ################################

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
    timer_marker = threading.Timer(5, look_for_marker)
    timer_marker.start()    
    ################ for the marker looking routine ###################
    # pub_land.publish(Empty_)


def custom_command(msg):
    global custom_height
    custom_height = msg.data/100
    print("The new Target height is !!! :  " + str(custom_height))    
    

def look_for_marker():
    global state_of_operation 
    state_of_operation = 0
    print("I could not find a marker lets look for it !!")
    # speed.linear.x = 0
    # speed.linear.y = 0
    # speed.linear.z = 0
    # speed.angular.z = 0.4
    # pub_move.publish(speed)

def main_algorithm(msg):
    global state_of_operation 
    global custom_x
    global custom_height

    print(state_of_operation)
    if state_of_operation == 0:
        print("Looking for marker...")
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.z = 0
        # speed.angular.z = 0.5
        # pub_move.publish(speed)

    if state_of_operation == 1:
        print("following the marker")
        custom_x = 0.2
        # speed.linear.x = 0
        # speed.linear.y = 0
        # speed.linear.z = 0
        # speed.angular.z = 0
        # pub_land.publish(Empty_)


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
    # global state_of_operation
    global timer_marker
    time.sleep(0.5)
    # pub_takeoff.publish(Empty_)
    time.sleep(2.5)
    rospy.Subscriber("/bebop/odom", Odometry, main_algorithm, queue_size=1)
    rospy.Subscriber("/visualization_marker", Marker, get_maker_pos, queue_size=1)

    rospy.Subscriber("/ar_pose_marker", Marker, get_maker_pos, queue_size=1)

    rospy.Subscriber("/bebop/odom", Odometry, callback, queue_size=1)
    rospy.Subscriber("/custom_command", Float32, custom_command, queue_size=1)

    timer_marker = threading.Timer(10,look_for_marker) # If 5 seconds elapse, call look for marker
    timer_marker.start()   
    print("hallo:")
    rospy.spin()

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()