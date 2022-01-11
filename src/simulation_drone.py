#!/usr/bin/env python2.7

from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float32
from visualization_msgs.msg import Marker
# from sensor_msgs.msg import CompressedImage
import math
import time
import rospy
import numpy as np
import csv

from drone_pid import PID_controller
# import PID_controller_class

# rot_Mat2 = np.array([[0, -1, 0],
#                     [1,  0, 0],
#                     [0,  0, 1]])

rot_Mat = np.array([[0, 0,  1],
                    [1, 0,  0],
                    [0, -1, 0]])

custom_height = 0.5


# tuning parameters
kp_v_x = 0.9
kp_v_y = 0.9
kp_v_z = 0.7    # changed
kp_av = 0.7

kd_v_x = 0.8
kd_v_y = 0.8
kd_v_z = 0.0    # changed
kd_av = 0.2

# needed for integrative part
ki_v_x = 0.6
ki_v_y = 0.6
ki_v_z = 0.2     # changed
ki_av = 0.02

ki_v_x_min = 0.1
ki_v_y_min = 0.1
ki_v_z_min = 0.1
ki_av_min = 0.1

ki_v_x_max = 0.9
ki_v_y_max = 0.9
ki_v_z_max = 0.9
ki_av_max = 0.9

PID_class = PID_controller(
    kp_v_x,  kp_v_y, kp_v_z, kp_av, ki_v_x, ki_v_y, ki_v_z, ki_av, kd_v_x,
    kd_v_y, kd_v_z, kd_av, ki_v_x_min, ki_v_y_min, ki_v_z_min, ki_av_min,
    ki_v_x_max, ki_v_y_max, ki_v_z_max, ki_av_max)

global goal_pos
goal_pos = Point()
theta_marker = 0

Empty_ = Empty()
speed = Twist()

rospy.init_node("speed_controller")
pub_takeoff = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pub_land = rospy.Publisher("/drone/land", Empty, queue_size=1)

PID_class = PID_controller(
    kp_v_x,  kp_v_y, kp_v_z, kp_av, ki_v_x, ki_v_y, ki_v_z, ki_av, kd_v_x,
    kd_v_y, kd_v_z, kd_av, ki_v_x_min, ki_v_y_min, ki_v_z_min, ki_av_min,
    ki_v_x_max, ki_v_y_max, ki_v_z_max, ki_av_max)

# def imput_points():
#     global goal
#     goal = Point()
#     goal.x = float(input("Input goal X position: "))
#     goal.y = float(input("Input goal Y position: "))
#     goal.z = float(input("Input goal Z position: "))


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


def callback(msg):
    global custom_height
    # global diff_ang
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # ##### code to follow the marker ############
    inc_x, inc_y, inc_z, distance_to_goal = dist(x, y, z)
    angle_to_goal = math.atan2(inc_y, inc_x)
    diff_ang = -diff_angels(theta, theta_marker)

    # print("the angel differace is", diff_ang)
    if theta_marker >= 0:
        diff_ang = (theta_marker - math.pi)
    if theta_marker < 0:
        diff_ang = (theta_marker + math.pi)

    # print("yaw = ", diff_ang)
    if abs(diff_ang) >= math.pi/2:
        diff_ang = 0

    # # if distance_to_goal > 0.1:
    # # PID_class.update_PID_contoller(inc_x, inc_y, inc_z, diff_ang)
  
    # PID_class.update_PID_contoller(
    #     goal_pos.x, goal_pos.y, goal_pos.z, diff_ang)

    # speed.linear.x = PID_class._linear_controler_x
    # speed.linear.y = PID_class._linear_controler_y
    # speed.linear.z = PID_class._linear_controler_z
    # speed.angular.z = -PID_class._angular_controler
    # # speed.angular.z = 0
    # # print(round(speed.angular.z, 3), "speed.angular.z")
    # # print(speed.linear.z)
    # pub_move.publish(speed)
    # ##### code to follow the marker ############

    ##### just responce to the the heigh command ############
    PID_class.update_PID_contoller(
    0, 0, custom_height-msg.position.z, 0)

    # print("the differences in the heights" + str(custom_height-msg.position.z))
    speed.linear.x = PID_class._linear_controler_x
    speed.linear.y = PID_class._linear_controler_y
    speed.linear.z = PID_class._linear_controler_z
    speed.angular.z = -PID_class._angular_controler



    pub_move.publish(speed)
    ##### just responce to the the heigh command ############
    ##### print to test csv ############
    with open('test_data.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([msg.position.z, speed.linear.z])
    ##### print to test csv ############




def get_maker_pos(msg):
    '''
    This function is called when the the drone sees a marker and
    returns a tartet (goal) postion for the drone
    '''
    global theta_marker
    global goal_pos
    # goal_pos = Point()

    (roll, pitch, yaw) = euler_from_quaternion(
                         [msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z,
                          msg.pose.orientation.w])

    vec_pos_mark_m_space = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]

    vec_pos_marker_g_space = np.dot((rot_Mat), vec_pos_mark_m_space)
    theta_marker = roll

    if theta_marker >= 0:
        diff_ang = -(theta_marker - math.pi)
    if theta_marker < 0:
        diff_ang = -(theta_marker + math.pi)

    goal_pos.x = vec_pos_marker_g_space[0] - math.cos(diff_ang)*0.5
    goal_pos.y = -vec_pos_marker_g_space[1] - math.sin(diff_ang)*0.5
    goal_pos.z = vec_pos_marker_g_space[2]

    ##### print to test csv ############
    with open('pos_marker.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([vec_pos_marker_g_space[0], -vec_pos_marker_g_space[1], vec_pos_marker_g_space[2]])
    ##### print to test csv ############

    print("............................")


def myhook():
    print("shutdown time!")
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    pub_land.publish(Empty_)


def custom_command(msg):
    global custom_height
    print(msg.data)
    custom_height = msg.data/100
    print(type(msg.data))


rospy.on_shutdown(myhook)

def main():
    time.sleep(1.5)
    pub_takeoff.publish(Empty_)
    sub_cam = rospy.Subscriber(
        "/visualization_marker", Marker, get_maker_pos, queue_size=1)

    # sub_inital = rospy.Subscriber(
    #     "/drone/goal_pose", Pose, get_goal_point, queue_size=1)
    # sub = rospy.Subscriber("/odom", Odometry, callbavisualization_markerck, queue_size=1)
    rospy.Subscriber("/custom_command", Float32, custom_command, queue_size=1)
    rospy.Subscriber("/drone/gt_pose", Pose, callback, queue_size=1)
    print("hallo:")
    rospy.spin()

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()