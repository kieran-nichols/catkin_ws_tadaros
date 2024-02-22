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
#rospy.Subscriber('europa_topic', EuropaMsg, handle_europa_input)
#rospy.Subscriber('sensing_topic', IMUDataMsg, handle_sensor_input)
#rospy.Subscriber('motor_listen', MotorListenMsg, listener)
        
#Publishing a topic
motor_pub = rospy.Publisher('motor_command', MotorDataMsg, queue_size=10)       

print("Type 'help' for description of full instructions")

#global variables
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
    cnts_per_rev = 567
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
    theta = theta_deg*math.pi/180
    beta = 5*math.pi/180
    q3 = 2*np.real((np.arccos(np.sin(theta/2)/np.sin(beta)))) # arccos in python always returns real values

    alpha = alpha_deg*math.pi/180
    # counterlockwise is positive 
    M1 = 180/math.pi*(alpha - np.arctan2(np.tan(q3/2),np.cos(beta))) 
    M2 = 180/math.pi*(-(alpha + np.arctan2(np.tan(q3/2), np.cos(beta))))
            
    #this is the code to find the rotation 
    #M1_prev is current position and can be any number
    #M1_new is what we want
    M1_new=M1; M2_new=M2
            
    temp1 = M1_new-prev_M1
    rot1 = temp1%360
    if rot1>180:
        rot1=rot1-360
                
    temp2 = M2_new-prev_M2
    rot2 = temp2%360
    if rot2>180:
        rot2=rot2-360
    
    #updating old
    prev_M1=M1; prev_M2=M2           
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
    motor1_angle = int(rot1*cnts_per_rev/360*motor_count_to_angles)
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

#the main loop that determines the action
def brain_action():
    while not rospy.is_shutdown():
        input_values = input_thread()
            
        if len(input_values)== 1:
            if input_values[0]=="k":
                os.system("rosnode kill -a")
                os.system("killall -9 rosmaster")
                print("ROS has been killed")
                
            elif var[0]=="help":                
                    print("\nTo command motor movement: 'm angle angle' ")
                
        elif len(input_values)== 2:
            continue
            
        elif len(input_values)== 3:
            if input_values[0] == "m":
                print("moving motor")
                publish_motor(input_values[1], input_values[2])
            if input_values[0] == "theta":
                print("theta and alpha to motor conversion")
                motor1_angle, motor2_angle = theta_alpha_to_motor_angles(input_values[1], input_values[2])
                #publish_motor(motor1_angle, motor2_angle)
            
                    
        else:
            print("Couldn't determine input")


        
       
if __name__ == '__main__':
    try:
        brain_action()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
