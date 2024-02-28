#!/usr/bin/env python
import rospy
import os
import time
import sys
import signal
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from tada_ros.msg import MotorDataMsg, MotorListenMsg, IMUDataMsg, EuropaMsg

import keyboard
import numpy as np
import math
import os
import threading
import psutil
import random

#Initializing variables
rospy.init_node('brain', anonymous=True)
        
#Subscribbing to some topics
print("init brain")

print("Type 'help' for description of full instructions")

#global variables
#imu
steps = 0

#previous values
prev_M1 = 0
prev_M2 = 0

# leg/ankle angle
PF = 0
EV = 0

#ratio of counts to angle
motor_count_to_angles = 0.087891
    
#getting foot angles
def theta_alpha_to_motor_angles(theta_deg, alpha_deg):
    #variables
    cnts_per_rev = 4096#567
    global motor_count_to_angles
    global prev_M1
    global prev_M2
    global PF
    global EV
    
    #transfering them to int
    theta_deg = int(theta_deg)
    alpha_deg = int(alpha_deg)
    
    #homed1 = self.homed1; homed2 = self.homed2
    #home_multiple1 = self.home_multiple1; home_multiple2 = self.home_multiple2 
    theta = theta_deg*math.pi/180  #converts to radians
    beta = 5*math.pi/180  #I think this is because wedges have a slope of 5 deg
    q3 = 2*np.real((np.arccos(np.sin(theta/2)/np.sin(beta)))) # arccos in python always returns real values
    alpha = alpha_deg*math.pi/180  #converts the degrees to radians
    # counterlockwise is positive
    M1 = 180/math.pi*(alpha - np.arctan2(np.tan(q3/2),np.cos(beta))) 
    M2 = 180/math.pi*(-(alpha + np.arctan2(np.tan(q3/2), np.cos(beta))))
            
    #this is the code to find the rotation 
    #M1_prev is current position and can be any number
    #M1_new is what we want
    
    rot1=M1
    rot2=M2
     
    # Wrapping function that ensures that the angle is between 180 and -180;
    ## need to finish verify
    # ~ M1 = (M1 + 180)%360 - 180
    # ~ M2 = (M2 + 180)%360 - 180
            
    q1 = M1*np.pi/180;
    q5 = M2*np.pi/180;
    q2 = np.pi/36; q4 = q2;
    R01 = np.array([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0], [0, 0, 1]])
    R12 = np.array([[np.cos(q2), 0 , np.sin(q2)], [0, 1, 0], [-np.sin(q2), 0, np.cos(q2)]])
    # ~ q3 = -q1 - q5;
    R23 = np.array([[np.cos(q3), -np.sin(q3), 0], [np.sin(q3), np.cos(q3), 0], [0, 0, 1]])
    R34 = np.array([[np.cos(q4), 0, np.sin(q4)], [0, 1, 0], [-np.sin(q4), 0, np.cos(q4)]])
    R45 = np.array([[np.cos(q5), -np.sin(q5), 0], [np.sin(q5), np.cos(q5), 0], [0, 0, 1]])
            
    R02 = np.matmul(R01,R12)
    R03 = np.matmul(R02,R23)
    R04 = np.matmul(R03,R34)
    R05 = np.matmul(R04,R45)
            
    PF = float(180/np.pi*np.arctan2(R05[0,2],R05[2,2])) #float(180/np.pi*R05[0,2])
    EV = float(180/np.pi*np.arctan2(R05[1,2],R05[2,2]))
            
    # Convert rotation in deg to rotation we need to move it in counts
    motor1_angle = int(rot1*cnts_per_rev/360*motor_count_to_angles) # cnts per rev* motor count to angles=1????
    #motor1_angle = int(rot1/360)
    motor2_angle = int(rot2*cnts_per_rev/360*motor_count_to_angles)
    
    print("PF:", PF, " EV:", EV, " Motor1 Angle:", motor1_angle, " Motor2 Angle:", motor2_angle)
    return (motor1_angle, motor2_angle)

#function that returns True if we need to stop/exit what we are doing
def to_stop():
    return keyboard.is_pressd('k')

#to format the time for consistancy
def format_time():
    current_time = rospy.Time.now()
    current_time_value = current_time.to_sec()
    current_time_value = current_time_value%100000
    return current_time_value

#to get input from command line
def input_thread():
    command_line = input("Enter command(s): ")
    raw_var = list(command_line.split())
    return raw_var

#to publish the motor
def publish_motor(motor1_angle, motor2_angle):
    motor_command = MotorDataMsg()
    motor_command.mode = 0
    motor_command.motor1_angle = int(motor1_angle)
    motor_command.motor2_angle = int(motor2_angle)
    motor_command.t = format_time()
    motor_pub.PF = PF
    motor_pub.EV = EV
    motor_pub.publish(motor_command)

#to update global variable steps
def update_from_imu(IMUDataMsg):
    global steps
    steps = IMUDataMsg.steps

#to make all possible combinations
def making_combinations(PF_angles, EV_angles):
    all_combinations = []
    for PF in PF_angles:
        for EV in EV_angles:
            all_combinations.append([PF, EV])
    return all_combinations

#waiting for certain amount of steps with ability to pause (?)
#TODO: add a way to pause
def waiting_for_steps(how_many_steps):
    global steps
    compare_step = steps
    while steps-compare_step < how_many_steps:
        continue
    return

#part of tada exp1, just the reoccuring portion
#how_wait defines whether we wait by step count or seconds
# how_wait = 1 means timer, how_wait = 2 means steps
def tada_exp_mini(theta, alpha, how_wait):
    step_count_walking = 6
    #step_count_relaxing = 3
    time_sec = 3
    
    #getting motor command
    motor1_angle, motor2_angle = theta_alpha_to_motor_angles(theta, alpha)
    publish_motor(motor1_angle, motor2_angle)
    #publish_motor(theta, alpha) #for testing, uncomment he line above, and comment this line
    
    if how_wait == 1:
        rospy.sleep(time_sec)
    elif how_wait == 2:
        #waiting for the amount of compare_step to happened
        waiting_for_steps(step_count_walking)
    
    #relaxing then
    #publish_motor(0, 0)
    #waiting_for_steps(step_count_relaxing)
    
    #done with this one experiment
    return
    
#tada exp1 with changing PF and EV, with relaxing steps in between
def tada_exp1():
    global steps

    alpha_angles = [-90, -45, 0, 45, 90]
    theta_angles = [-10, -5, 0, 5, 10]
    all_combinations = making_combinations(theta_angles, alpha_angles)
    #all_combinations = [[theta, alpha], [theta, alpha],.....]
    print("Angles [theta, alpha]: ", all_combinations)
    
    #taking random PF and EV
    while len(all_combinations) > 0:
        random_element = random.choice(all_combinations) #choosing a random angle
        tada_exp_mini(random_element[0], random_element[1], 2) #doing an experiment with that angle
        all_combinations.remove(random_element) #deleting that angle
    
    print("End of experiment 1")
    return

#tada exp for mocap
def tada_exp_mocap():
    alpha_angles = [-90, -45, 45, 90]
    theta_angles = [-10, -5, 5, 10]
    all_combinations = making_combinations(theta_angles, alpha_angles)
    #all_combinations = [[theta, alpha], [theta, alpha],.....]
    print("Angles [theta, alpha]: ", all_combinations)
    
    #taking random PF and EV
    while len(all_combinations) > 0:
        random_element = random.choice(all_combinations) #choosing a random angle
        tada_exp_mini(random_element[0], random_element[1], 1) #doing an experiment with that angle
        all_combinations.remove(random_element) #deleting that angle
    
    print("End of mocap")
    return

#the main loop that determines the action
def brain_action():  
    while not rospy.is_shutdown():
        input_values = input_thread()
            
        if len(input_values)== 1:
            if input_values[0]=="k":
                os.system("rosnode kill -a")
                os.system("killall -9 rosmaster")
                print("ROS has been killed")
            elif input_values[0]=="exp1":
                print("Experiment 1")
                tada_exp1()
            elif input_values[0]=="mocap":
                print("MOCAP Experiment")
                tada_exp_mocap()
            elif input_values[0]=="help":                
                print("\nTo command motor movement: 'm angle angle' ")
                
        elif len(input_values)== 2:
            continue
            
        elif len(input_values)== 3:
            if input_values[0] == "m":
                print("moving motor")
                publish_motor(input_values[1], input_values[2])
            elif input_values[0] == "theta":
                print("theta and alpha to motor conversion")
                motor1_angle, motor2_angle = theta_alpha_to_motor_angles(input_values[1], input_values[2])
                publish_motor(motor1_angle, motor2_angle)             
        else:
            print("Couldn't determine input")

#putting hre so that all subscriber functions have been defined
#Publishing a topic
motor_pub = rospy.Publisher('motor_command', MotorDataMsg, queue_size=10)

#Getting subscribers
rospy.Subscriber('sensing_topic', IMUDataMsg, update_from_imu)
        
       
if __name__ == '__main__':
    try:
        brain_action()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
