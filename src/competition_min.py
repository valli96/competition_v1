#!/usr/bin/env python2.7

## ros import
import rospy
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Float32, Header
from visualization_msgs.msg import Marker
from tf import TransformListener, Transformer
from ar_track_alvar_msgs.msg  import AlvarMarkers
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged



# /bebop/states/ardrone3/PilotingState/AltitudeChanged
# /bebop/states/ardrone3/PitingState/AltitudeChanged

## general imports
from tf.transformations import decompose_matrix, euler_from_quaternion
from os import spawnlp
import csv
from numpy import genfromtxt
import threading
import math
import time
import numpy as np
from simple_pid import PID

### global variables


## positions before rotation
#
#

## positions after rotaions 
#
#

state_of_operation = 0 # 0 starting Drone 
                       # 1 looking for the marker 
                       # 2 following the marker
                       # 3 landing on the marker
                       # 4 looking for the marker 
                       # 5 following the marker
                       # 6 landing on the marker
                       # 7 looking for the marker 
                       # 8 following the marker

                       # 10 landing without visual 

marker_visible = False
ground_vehicle_stopped = False
target_height_reached = False
first_run_target_height = 0
last_side_marker = "left"
first_time_state_10 = True

marker_position = Point()
Empty_ = Empty()
speed = Twist()
pose_drone = Pose()

pose_marker_accurate = [0,0,0]
current_velocities = [0,0,0]
initial_odom = [0,0,0]
actual_drone_pose = [0,0,0]
target_height = 1.5
speed_limit = 0.5


rospy.init_node("controll_drone")

pub_takeoff = rospy.Publisher("bebop/takeoff", Empty, queue_size=1)
pub_land = rospy.Publisher("/bebop/land", Empty, queue_size=1)
camera_controll = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
pub_move = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
Empty_ = Empty()

pid_x_fast = PID(0.7,0.15,0, setpoint=0, sample_time=0.01)
pid_y_fast = PID(0.3,0,0, setpoint=0, sample_time=0.01)
pid_z_fast = PID(0.3,0.2,0, setpoint=0, sample_time=0.01)
pid_rot_fast = PID(0.5,0.15,0, setpoint=0, sample_time=0.01)

pid_x_slow = PID(1,0,0, setpoint=0, sample_time=0.01) 
pid_y_slow = PID(1,0,0, setpoint=0, sample_time=0.01) 
pid_z_slow = PID(0.1,0,0, setpoint=0, sample_time=0.01)
pid_rot_slow = PID(0.5,0.15,0, setpoint=0, sample_time=0.01)

pid_x_2 = PID(1,0,0, setpoint=0, sample_time=0.01) 
pid_y_2 = PID(1,0,0, setpoint=0, sample_time=0.01) 
pid_z_2 = PID(1,0,0, setpoint=0, sample_time=0.01)


def publish_speed_to_drone(speed):
    global state_of_operation 

    if state_of_operation == 12:
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.z = 0

    # if abs(speed.linear.x) > speed_limit:
    #     speed.linear.x = 0
    # if abs(speed.linear.y)> speed_limit:
    #     speed.linear.y = 0
    # # if abs(speed.linear.z) > speed_limit:
    # #     speed.linear.y = 0
    # if abs(speed.angular.z) > speed_limit:
    #     speed.angular.z = 0
    if state_of_operation != 12:
        pub_move.publish(speed)
        pass



def get_maker_pose(msg):
    if msg.markers:
        global state_of_operation
        global marker_visible
        global timer_marker

        marker_visible = True

        # marker find in first stage (following)
        if state_of_operation == 1 or state_of_operation == 0 and target_height_reached == True:
            state_of_operation = 2 


        ######################### position ################################
        vec_pos_mark_m_space = [msg.markers[0].pose.pose.position.x,
                                msg.markers[0].pose.pose.position.y,
                                msg.markers[0].pose.pose.position.z]

        marker_position.x = vec_pos_mark_m_space[0] # - 0.2 #- math.cos(diff_ang)*0.5
        marker_position.y = vec_pos_mark_m_space[1] # + 0.2 #- math.sin(diff_ang)*0.5
        marker_position.z = vec_pos_mark_m_space[2]
        print(vec_pos_mark_m_space)
        ######################### position ################################

        ################ for the marker looking routine ###################
        timer_marker.cancel()
        timer_marker = threading.Timer(1.5, look_for_marker)
        timer_marker.start()    
        ################ for the marker looking routine ###################
        with open('goal_pos.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([marker_position.x, marker_position.y, marker_position.z])
        

def look_for_marker():
    global state_of_operation
    global marker_visible
    global last_side_marker

    marker_visible = False

    pos_data = genfromtxt('goal_pos.csv', delimiter=',')
    if pos_data[-1,1] < 0:
        last_side_marker = "left"
    if pos_data[-1,1] >= 0:
        last_side_marker = "right"
    
    print(last_side_marker)

    if (state_of_operation == 0 or state_of_operation == 2):
        state_of_operation = 1  
        # state_of_operation = 4

def get_current_velocities(msg):
    global current_velocities
    current_velocities[0]= msg.speedX
    current_velocities[1]= msg.speedY 
    current_velocities[2]= msg.speedZ 

 
def detect_ground_vehicle_stop():
    global ground_vehicle_stopped
    global pose_marker_accurate
    while (ground_vehicle_stopped == False):
        pose_data = genfromtxt('goal_pos.csv', delimiter=',')
        average_x = sum(abs(pose_data[0,:-5]))/5
        average_y = sum(abs(pose_data[1,:-5]))/5
        average_z = sum(abs(pose_data[2,:-5]))/5
        if (average_x < 10 and average_y < 10):
            ground_vehicle_stopped = True
            pose_marker_accurate[0] = average_x
            pose_marker_accurate[1] = average_y
            pose_marker_accurate[2] = average_z


# def move_target_height(msg):
#     global target_height_reached
#     global first_run_target_height
#     global initial_odom
#     global target_height
 
#     if first_run_target_height == 4:
#         initial_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
#         # first_run_target_height = first_run_target_height + 1
#     first_run_target_height = first_run_target_height + 1

#     if target_height_reached == False:
#         speed.linear.z = pid_z_fast(((msg.pose.pose.position.z-initial_odom[2]) - target_height))
#         speed.linear.x = 0
#         speed.linear.y = 0
#         speed.angular.z = 0

#         if abs((msg.pose.pose.position.z-initial_odom[2]) - target_height) < 0.2:
#             target_height_reached = True
#         print("bothe:  " + str((msg.pose.pose.position.z-initial_odom[2])))
#         print("inside: "+ str(((msg.pose.pose.position.z-initial_odom[2]) - target_height)))
#         print("speed:  " + str((speed.linear.z)))


        publish_speed_to_drone(speed)


def move_target_height_2(msg):
    global target_height_reached
    global target_height
    global state_of_operation
    # print("______ Paul ich bin hier ________________")
 
    if target_height_reached == False:
        speed.linear.z = pid_z_fast(((msg.altitude) - target_height))
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = 0

        if abs((msg.altitude) - target_height) < 0.3:
            target_height_reached = True
            state_of_operation = 1
            speed.linear.z = 0
            print("_______________________________________________________________________________----")

        print("both:  " + str(((msg.altitude))))
        print("inside: "+ str(-((msg.altitude) - target_height)))
        print("speed:  " + str((speed.linear.z)))

        publish_speed_to_drone(speed)


#######################################################################################
#######################################################################################
def main_algorithm(msg):
    global current_velocities
    global state_of_operation
    global last_side_marker
    global first_time_state_10
    global pose_marker_accurate
    global pid_x_fast, pid_y_fast, pid_z_fast
    global pid_x_slow, pid_y_slow, pid_z_slow
    global pid_x_2, pid_y_2, pid_z_2

    with open('velocity_drone.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([speed.linear.x, speed.linear.y,  speed.angular.z])
    
    with open('velocity_drone_real.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([current_velocities[0], current_velocities[1], current_velocities[2]])

    # print("State of the system "+str(state_of_operation))
    with open('current_state.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow([state_of_operation])
        
######################### State 1 ###############################
    if state_of_operation == 1 and target_height_reached == True:
        print("ich bin im state 1:   ")
       
        if last_side_marker == "right":
            speed.angular.z = +0.2

        elif last_side_marker == "left":
            speed.angular.z = -0.2

        speed.linear.z = 0
        publish_speed_to_drone(speed)
######################### State 1 ###############################

######################### State 2 ###############################
    # if state_of_operation == 2:
    #     speed.linear.x = pid_x_fast(-(marker_position.x - 2))
    #     speed.linear.y = pid_y_fast(-(marker_position.y))
    #     speed.linear.z = 0
    #     speed.angular.z = pid_rot_fast(-marker_position.y)

    #     speed.linear.x = pid_x_2(-(speed.linear.x - current_velocities[0]))  ### my second PID controller
    #     speed.linear.y = pid_y_2((speed.linear.y - current_velocities[1]))
    #     speed.linear.z = 0

    #     publish_speed_to_drone(speed)

    #     detect_ground_vehicle_stop()  ### also returns exact position

    #     if (ground_vehicle_stopped == True and marker_position.x < 1.5 and marker_position.y < 0.3):
    #         state_of_operation = 10
    #         # pub_land.publish(Empty_)
######################### State 2 ###############################

######################### State 10 ###############################
    # if state_of_operation == 10:
    #     if first_time_state_10 == True:
    #         actual_drone_pose[0] = msg.pose.pose.position.x
    #         actual_drone_pose[1] = msg.pose.pose.position.y
    #         actual_drone_pose[2] = msg.pose.pose.position.z
    #         first_time_state_10 = False


    #     speed.linear.x = pid_x_slow(-(msg.pose.pose.position.x-actual_drone_pose[0])- pose_marker_accurate[0])
    #     # speed.linear.y = pid_y_(-(pose_marker_accurate))
    #     speed.linear.y = 0
    #     speed.angular.z = pid_rot_slow(-pose_marker_accurate[1])

    #     speed.linear.x = pid_x_2(-(speed.linear.x - current_velocities[0]))  ### my second PID controller
    #     # speed.linear.y = pid_y_2((speed.linear.y - current_velocities[1]))
    #     speed.linear.z = 0

    #     publish_speed_to_drone(speed)
    #     if ((msg.pose.pose.position.x-actual_drone_pose[0])- pose_marker_accurate[0]) < 0.1:
    #         state_of_operation = 8
######################## State 3 ###############################

#######################################################################################
#######################################################################################
def main():
    global state_of_operation
    global timer_marker
    print("Hallo")
    # Empty_ = Empty()
    time.sleep(0.5)
    pub_takeoff.publish(Empty_)
    time.sleep(2.5)
    
    rospy.Subscriber("/bebop/odom", Odometry, main_algorithm, queue_size=1)     # main function (state machine)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, move_target_height_2, queue_size=1)



    # rospy.Subscriber("/bebop/odom", Odometry, move_target_height, queue_size=1)     # main function (state machine)
    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_maker_pose, queue_size=1)  # get marker position
    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_maker_pose, queue_size=1)  # get marker position


    # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, get_current_velocities, queue_size=1)  # needed for the seconde PID
    # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, get_current_velocities, queue_size=1)  # needed for the seconde PID


    timer_marker = threading.Timer(3,look_for_marker)  
    timer_marker.start() 
    rospy.spin()

###################### if the process is killed ##########################################
def myhook():
    global state_of_operation 
    state_of_operation = 12
    # time.sleep(0.5)
    pub_land.publish(Empty_)
    time.sleep(0.5)
    print("shutdown time!")
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0 
    state_of_operation = 12
    # pub_move.publish(speed)
    print("program aborted drone will land")
    pub_land.publish(Empty_)
    time.sleep(3)

rospy.on_shutdown(myhook)
##################### if the process is killed ##########################################


if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()