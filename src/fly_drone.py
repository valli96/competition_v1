#!/usr/bin/env python2.7

## ros related imports
# from tkinter.tix import Tree
# from tkinter.tix import Tree
# from gui import marker_detection
# from multiprocessing import log_to_stderr
# from turtle import distance
import rospy
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, Float32, Header
from visualization_msgs.msg import Marker
from tf import TransformListener, Transformer
from ar_track_alvar_msgs.msg  import AlvarMarkers
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged


# from sensor_msgs.msg import CompressedImage

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



# if True jump to state 10
# if False jump to state 3
land_without_visual_feedback = True
pose_drone_accurate = [0,0,0]


goal_pos = Point()
Empty_ = Empty()
speed = Twist()

marker_visible_stage_2 = False

current_Odom = Odometry()

### variables for opto-track implementation 
pose_drone = Pose()


marker_visible = False
camera_direction = Twist()
marker_detected_stage_2 = False


rospy.init_node("speed_controller")

# rostopic pub rostopic pub /bebop/camera_control geometry_msgs/Twist 

camera_controll = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

pub_takeoff = rospy.Publisher("bebop/takeoff", Empty, queue_size=1)
pub_move = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
pub_land = rospy.Publisher("/bebop/land", Empty, queue_size=1)

#########################################################################################
def publish_speed_to_drone(speed):
    if abs(speed.linear.x) > speed_limit:
        speed.linear.x = np.sign(speed.linear.x)* speed_limit

    if abs(speed.linear.y)> speed_limit:
        speed.linear.y = np.sign(speed.linear.y)* speed_limit

    if abs(speed.linear.z) > speed_limit:
        speed.linear.z = np.sign(speed.linear.z)* speed_limit

    if abs(speed.angular.z) > speed_limit:
        speed.angular.z = np.sign(speed.angular.z)* speed_limit

    pub_move.publish(speed)
#########################################################################################

# def diff_angels(x, y):
#     global theta_marker
#     x += math.pi
#     y += math.pi
#     a = (x - y) % (math.pi*2)
#     b = (y - x) % (math.pi*2)
#     return -a if a < b else b

pid_x = PID(0.7, 0.1, 0.01, setpoint=0)
pid_y = PID(0.05, 0.0, 0.00, setpoint=0)    
pid_z = PID(0.05, 0.01, 0.05, setpoint=0)
pid_rot = PID(0.3, 0.02, 0.00, setpoint=0)

ground_vehicle_stopped = False

diff_ang = 0
first_run = True
initial_odom = [0, 0, 0]  # [x , y , z]
initial_angel = 0

yaw_marker = 0
speed_limit = 0.35
custom_height = 1
custom_x = 0.0
custom_y = 0.0
custom_angel = 50*np.pi/180
last_side_marker = "left"  #  left or right are possible

current_velocities = [0,0,0]


# def callback(msg):

#     '''
#     this function is used for debugging in moving the drone to arbitrary positions
#     the first iteration of algorithm don't the the postion of the drone!  

#     '''
#     global first_run
#     global initial_odom
#     global initial_angel
#     global custom_x
#     global custom_y
#     global custom_height
#     global custom_angel
#     global yaw_marker
#     global state_of_operation
#     global pose_drone

    # if state_of_operation == 2:
        
        ################################################################################
        ############ this was based on the odom topic of the drone
        ################################################################################

        ################## odom message of drone ###############################
        # pos_done = [msg.pose.pose.position.x,
        #             msg.pose.pose.position.y,
        #             msg.pose.pose.position.z]

        # rot_q = msg.pose.pose.orientation
        # (roll, pitch, yaw_drone) = euler_from_quaternion(
        #     [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # if yaw_drone > 0:
        #     yaw_drone = yaw_drone - math.pi
        # elif yaw_drone < 0:
        #     yaw_drone = yaw_drone + math.pi
        # ################## odom message of drone ################################

        # ################################### calibration IMU ######################
        # if first_run == True:
        #     initial_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        #     initial_angel = yaw_drone
        #     first_run = False
        # ################################### calibration IMU ######################

        # ##################### adjust to target pos and pub speed ######################
        # # pos_done[0] = (msg.pose.pose.position.x - initial_odom[0]) - custom_x # custom_x
        # # pos_done[1] = (msg.pose.pose.position.y - initial_odom[1]) - custom_y
        # # pos_done[2] = (msg.pose.pose.position.z - initial_odom[2]) - custom_height  # - 2 the number will define how heigh the drone can hover 
        # # # drone_rot = (msg.pose.pose.rotaion.z - initial_odom[3]) - initial_angel  #  this is the yaw of the drone

        # # publish_speed_to_drone(speed)
        # ##################### adjust to target pos and pub speed #####################
        # with open('velocity_drone.csv', 'a') as f:
        #     writer = csv.writer(f)
        #     writer.writerow([speed.linear.x, speed.linear.y, speed.angular.z])

        ################################################################################
        ############ this was based on the pose value from opto-track
        ################################################################################

        # pose_difference.position.x = pose_jackal.position.x - pose_drone.position.x
        # pose_difference.position.y = pose_jackal.position.y - pose_drone.position.y
        # pose_difference.position.z = pose_jackal.position.z - pose_drone.position.z 



# def custom_command(msg):
#     global custom_height
#     custom_height = msg.data/100
#     print("The new Target height is !!! :  " + str(custom_height))
    

def look_for_marker():
    """
    function is called after the marker can not be detected any more
    -> finds the last detected position of the marker
    -> changes the state for looking after the marker 
    """
    global last_side_marker
    global state_of_operation
    global marker_visible
    global marker_detected_stage_2

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

    if state_of_operation == 5 and  marker_detected_stage_2 == False:
        state_of_operation = 6

    if state_of_operation == 5 and  marker_detected_stage_2 == True:
        state_of_operation = 7



def get_maker_pose(msg):
    if msg.markers:
        # print("I see the the marker")
        global diff_ang
        global timer_marker
        global state_of_operation
        global goal_pos
        global yaw_marker
        global marker_visible
        global marker_detected_stage_2

        marker_visible = True

        # marker find in first stage (following)
        if state_of_operation == 1 or state_of_operation == 0:
            state_of_operation = 2 

        # marker find in seconde stage (landing)
        if state_of_operation == 6 or state_of_operation == 7 or state_of_operation == 4:
            state_of_operation = 5
            marker_detected_stage_2 = True

        ######################### position ################################
        vec_pos_mark_m_space = [msg.markers[0].pose.pose.position.x,
                                msg.markers[0].pose.pose.position.y,
                                msg.markers[0].pose.pose.position.z]

        goal_pos.x =  vec_pos_mark_m_space[0] # - 0.2 #- math.cos(diff_ang)*0.5
        goal_pos.y = -vec_pos_mark_m_space[1] # + 0.2 #- math.sin(diff_ang)*0.5
        goal_pos.z = vec_pos_mark_m_space[2]
        print(vec_pos_mark_m_space)
        ######################### position ################################

        ######################### rotation ################################
        (pitch, roll, yaw_marker) = euler_from_quaternion(
            [msg.markers[0].pose.pose.orientation.x,
             msg.markers[0].pose.pose.orientation.y, 
             msg.markers[0].pose.pose.orientation.z, 
             msg.markers[0].pose.pose.orientation.w])
        # rotation offest fixed. before 90 now 0 
        if yaw_marker > -math.pi/2:
            yaw_marker = yaw_marker - math.pi/2
        elif yaw_marker < -math.pi/2:
            yaw_marker = yaw_marker + 3*math.pi/2
        ######################### rotation ################################
        
        ################ writing to csv File ##############################
        # with open('goal_pos.csv', 'a') as f:
        #         writer = csv.writer(f)
        #         writer.writerow([goal_pos.x, goal_pos.y,goal_pos.z, yaw_marker])
        ############### writing to csv File ###############################

        ################ for the marker looking routine ###################
        timer_marker.cancel()
        timer_marker = threading.Timer(1.5, look_for_marker)
        timer_marker.start()    
        ################ for the marker looking routine ###################
        with open('goal_pos.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([goal_pos.x, goal_pos.y, goal_pos.z])
        

# def get_pose_opto_track_drone(msg):
#     global pose_drone
    

def get_current_velocities(msg):
    global current_velocities
    current_velocities[0]= msg.speedX
    current_velocities[1]= msg.speedY 
    current_velocities[2]= msg.speedZ 

def detect_ground_vehicle_stop():
    global ground_vehicle_stopped
    if state_of_operation == 2:
        pos_data = genfromtxt('goal_pos.csv', delimiter=',')
        average_x = sum(abs(pos_data[0,:-20]))
        average_y = sum(abs(pos_data[0,:-20]))


        if (average_x < 10 and average_y < 10):
            ground_vehicle_stopped = True


def main_algorithm(msg):
    global state_of_operation 
    global custom_x
    global custom_height
    global diff_ang
    global goal_pos
    global last_side_marker
    global current_velocities
    global pid_x, pid_y, pid_z
    global ground_vehicle_stopped
    global initial_odom
    global first_run
    global marker_visible_stage_2
    global pose_drone_accurate 
    ######

    if first_run == True:
        initial_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        first_run = False

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
    if state_of_operation == 1:
        # print("Looking for marker...")
        
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        if last_side_marker == "right":
               speed.angular.z = -0.2

        elif last_side_marker == "left":
            speed.angular.z = 0.2

        publish_speed_to_drone(speed)
    ######################### State 1 ###############################

    ######################### State 2 ###############################
    if state_of_operation == 2:

        pid_x = PID(0.3, 0.04, 0.01, setpoint=0)
        pid_y = PID(0.2, 0.0, 0.00, setpoint=0)    
        # pid_z = PID(0.05, 0.01, 0.05, setpoint=0)
        # pid_rot = PID(0.3, 0.02, 0.00, setpoint=0)

        # speed.linear.x = pid_x(-(goal_pos.x - 1.5))
        speed.linear.y = pid_y(-(goal_pos.y))
        # speed.linear.z = 0
        # speed.angular.z = pid_rot(diff_ang)

        print("Speed before second pid"+  str(speed.linear.x))
        pid_x_2 = PID(2, 0.15, 0.01, setpoint=0)           ### my second PID controller
        pid_y_2 = PID(0.5, 0.05, 0.01, setpoint=0)           ### my second PID controller
        # speed.linear.x = pid_x_2(-(speed.linear.x - current_velocities[0]))  ### my second PID controller
        speed.linear.y = pid_y_2((speed.linear.y - current_velocities[1]))
        print("Speed after second pid"+  str(speed.linear.x))
        # speed.linear.x = 0
        # speed.linear.y = 0
        speed.linear.z = 0
        # speed.angular.z = 0
        # speed.angular.z = pid_rot(goal_pos.y)

        publish_speed_to_drone(speed)

        # timer_ground_vehicle_stops = threading.Timer(3,detect_ground_vehicle_stop) # wait 5 seconds before starting to look for the marker 
        # timer_marker.start() #velocities


    ### assumption ground vehicle has stopt 
        if (ground_vehicle_stopped == True and goal_pos.x < 1.5 and goal_pos.y < 0.3): # 
            state_of_operation = 3
            print("Ich bin hier")
            pub_land.publish(Empty_)

    ######################### end State 2 ##############################

    ######################### State 3 ##############################
    if state_of_operation == 3:
        if marker_visible == True:
            speed.linear.x = speed.linear.x = pid_x(-(goal_pos.x))*0.3 ## move in direction of marker until it is not visible any more
            speed.linear.y = pid_y((goal_pos.y))
            speed.angular.z = pid_rot(goal_pos.y)
            publish_speed_to_drone(speed)

        ## move torwards the marker without feedback.  Speed and time need to be tuned 
        if marker_visible == False:
            speed.linear.x = 0.1
            speed.linear.y = 0
            speed.angular.z = 0
            time.sleep(2)
            state_of_operation == 4

        ## moving without visual feedback use feedback by integrating the velocity values over time
        ## implementation with opto-track !!!
    ######################### end State 3 ##############################

    ######################### State 4 ##############################
    if state_of_operation == 4:
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.z = 0
        publish_speed_to_drone(speed)
        camera_direction.angular.y = -90
        camera_controll.publish(camera_direction)
        time.sleep(0.5)
        state_of_operation = 5
    ######################### end State 4 ##############################

    ######################### State 5 ##############################
    if state_of_operation == 5:
        pid_x = PID(0.7, 0.1, 0.01, setpoint=0)
        pid_y = PID(0.5, 0.0, 0.00, setpoint=0)
        # pid_z = PID(0.5, 0.01, 0.05, setpoint=0)
        # pid_rot = PID(0.3, 0.02, 0.00, setpoint=0)

    # ######################################################################
    # # this next section need to be switched manually depending on the drone  
    # ######################################################################
        if True:
            speed.linear.x = pid_x(-(goal_pos.x))*0.5   # since the camera calibration is wrong  
            speed.linear.y = pid_y((goal_pos.y))*0.5    # since the camera calibration is wrong
            pid_2 = PID(0.7, 0.15, 0.01, setpoint=0)           ### my second PID controller
            speed.linear.x = pid_2(-(speed.linear.x - current_velocities[0]))  ### my second PID controller
            speed.linear.y = pid_2(-(speed.linear.x - current_velocities[0]))  ### my second PID controller

        if False:
            speed.linear.x = pid_x(-(goal_pos.z))*0.5   # since the camera calibration is wrong  
            speed.linear.y = pid_y((goal_pos.y))*0.5    # since the camera calibration is wrong

        # speed.linear.z = pid_z(-(goal_pos.z - 0.30)) # sign and distance need to be checked 
        speed.angular.z = 0
        publish_speed_to_drone(speed)
           

        if (abs(goal_pos.x) < 0.10 and abs(goal_pos.y) < 0.10 and goal_pos.z < 0.5  and goal_pos.x != 0): #  goal_pos.x != 0 is just for testing
            state_of_operation = 8
    ######################### end State 5 ##############################

    ######################### State 6 ##############################
    if state_of_operation == 6:
        ## move up for some random time 
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = 0
        speed.linear.z = pid_z((msg.pose.pose.postion.z - initial_odom[2]) - 2) ## goes to 2 m
        # publish_speed_to_drone(speed)
    ######################### end State 6 ##############################
    
    ######################### State 7 ##############################
    if state_of_operation == 7:
        ## move up for some random time 
        # go in the direction of the last know position 
        last_pose_marker = genfromtxt('goal_pos.csv', delimiter=',')
        speed.linear.x = pid_x(-last_pose_marker) 
        speed.linear.y = pid_y(last_pose_marker)

        publish_speed_to_drone(speed)
    ######################### end State 7 ##############################

    ######################### State 8 ##############################
    if state_of_operation == 8:
        # print("Landing ")
        pub_land.publish(Empty_)
    ######################### end State 8 ##############################


    ######################### State 10 ##############################
    if state_of_operation == 10:
        get_accurate_ground_vehicle_pose()
        speed.linear.x = pid_x()
        speed.linear.y = pid_y()
        speed.linear.z = pid_z()
    ######################### end State 10 ##############################

def get_accurate_ground_vehicle_pose():
    global pose_drone_accurate 
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.z = 0
    # publish_speed_to_drone(speed)
    time.sleep(4)
    data =  np.array(genfromtxt('goal_pos.csv', delimiter=','))
    pose_drone_accurate[0] = sum(abs(data[-20:,0]))/20
    pose_drone_accurate[1] = sum(abs(data[-20:,1]))/20
    pose_drone_accurate[2] = sum(abs(data[-20:,2]))/20
 


def main():
    # global state_of_operation
    global timer_marker
    time.sleep(0.5)
    pub_takeoff.publish(Empty_)
    time.sleep(2.5)

    rospy.Subscriber("/bebop/odom", Odometry, main_algorithm, queue_size=1)     # main function (state machine)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_maker_pose, queue_size=1)  # get marker position
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, get_current_velocities, queue_size=1)  # needed for the seconde PID

    # rospy.Subscriber("/bebop/odom", Odometry, callback, queue_size=1)         # is used for debugging and controll the drone without the marker
    # rospy.Subscriber("/custom_command", Float32, custom_command, queue_size=1)  # resice and handels commands from the UI
    # rospy.Subscriber("/bebop/odom", AlvarMarkers, em, queue_size=1)  # get marker positions
    # rospy.Subscriber("*/drone_1", Pose, get_pose_opto_track_drone, queue_size=1)
    
    timer_marker = threading.Timer(3,look_for_marker) # wait 5 seconds before starting to look for the marker 
    timer_marker.start() #velocities

    # timer_ground_vehicle_stops = threading.Timer(3,detect_ground_vehicle_stop) # wait 5 seconds before starting to look for the marker 
    # timer_marker.start() #velocities


    print("hallo:")
    rospy.spin()


###################### if the process is killed ##########################################
def myhook():
    global state_of_operation
    pub_land.publish(Empty_)
    print("shutdown time!")
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    pub_move.publish(speed)
    state_of_operation = 8
    time.sleep(2)
    print("program aborted drone will land")

rospy.on_shutdown(myhook)
##################### if the process is killed ##########################################


if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()