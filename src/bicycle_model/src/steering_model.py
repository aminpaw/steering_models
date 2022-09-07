#! /bin/env python3
import rospy
from math import *
from geometry_msgs.msg import Twist
from bicycle_model.srv import Inputs
import time
import threading

msg_front = Twist()
msg_rear = Twist()

def move_front(vel,delta,time_interval,length_f,length_r):
    ##### CALCULATES BETA ANGLE #####
    def calc_beta(length_r,length_f,delta) :
        beta = atan((length_r * tan(delta))/(length_f+length_r))
        return beta

    ##### CALCULATES ANGULAR VELOCITY #####
    def calc_theta_dot(vel,beta,delta,length_f,length_r):
        theta_dot = (vel*cos(beta)*tan(delta))/(length_f+length_r)
        return theta_dot

    ##### INITIAL STATE #####
    Time = Time_Start =  time.time()    # stores initial time 
    init_delta = delta                  # stores initial steering angle

    while ((Time-Time_Start) < time_interval):      # check if time spent is less than time of motion
        ##### CALCULATE OUTPUTS #####
        beta = calc_beta(length_r,length_f,delta)
        theta_dot = calc_theta_dot(vel,beta,delta,length_f,length_r)

        ##### OUTPUT UPDATED STATE #####
        msg_front.linear.x,msg_front.angular.z = vel,theta_dot
        pub_front.publish(msg_front)

        ##### DECAY STEERING ANGLE #####
        delta = init_delta*exp(-0.5*(Time-Time_Start))

        ##### UPDATE TIME #####
        Time = time.time()

def move_rear(vel,delta,time_interval,length_f,length_r):
    ##### CALCULATES BETA ANGLE #####
    def calc_beta(length_r,length_f,delta) :
        beta = atan(((length_f+length_r) * tan(delta))/(length_f))
        return beta

    ##### CALCULATES ANGULAR VELOCITY #####
    def calc_theta_dot(vel,beta,delta,length_f,length_r):
        theta_dot = (vel*sin(beta))/(length_r)
        return theta_dot

    ##### INITIAL STATE #####
    Time = Time_Start =  time.time()    # stores initial time 
    init_delta = delta                  # stores initial steering angle
    
    while ((Time-Time_Start) < time_interval):      # check if time spent is less than time of motion
        ##### CALCULATE OUTPUTS #####
        beta = calc_beta(length_r,length_f,delta)
        theta_dot = calc_theta_dot(vel,beta,delta,length_f,length_r)

        ##### OUTPUT UPDATED STATE #####
        msg_rear.linear.x,msg_rear.angular.z = vel,theta_dot
        pub_rear.publish(msg_rear)

        ##### DECAY STEERING ANGLE #####
        delta = init_delta*exp(-0.5*(Time-Time_Start))

        ##### UPDATE TIME #####
        Time = time.time()

def control(request):
    ##### INPUTS #####
    """
    Takes input values from service requests
    """
    vel = request.velocity          # lateral velocity 
    delta = request.delta           # steering angle
    time_interval = request.time    # time of motion

    ##### PARAMAETERS #####
    """
    Takes car parameters from yaml file
    """
    length_r = rospy.get_param("/Lengths/Rear")     # length from rear wheel to center of gravity
    length_f = rospy.get_param("/Lengths/Front")    # length from front wheel to center of graviy

    ##### SIMULATES MODEL BASED ON INPUT ARGUMENT #####
    if request.model == 0:      ## RUNS FRONT STEERING MODEL
        move_front(vel,delta,time_interval,length_f,length_r)

    if request.model == 1:      ## RUNS REAR STEERING MODEL
        move_rear(vel,delta,time_interval,length_f,length_r)

    
    if request.model == 2:      ## RUNS BOTH MODELS SIMULTANEOUSLY           
        process1 = threading.Thread(target=move_front,args=(vel,delta,time_interval,length_f,length_r))
        process2 = threading.Thread(target=move_rear,args=(vel,delta,time_interval,length_f,length_r))

        process1.start()
        process2.start()

        process1.join()
        process2.join()
        
    return "Done!"


rospy.init_node("Controller")
pub_front = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size =10)
pub_rear = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size =10)
service = rospy.Service("Inputs", Inputs, control)
rospy.spin()