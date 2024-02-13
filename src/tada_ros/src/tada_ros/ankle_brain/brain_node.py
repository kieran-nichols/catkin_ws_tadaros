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
                publish_motor(input_values[1], input_values[2])
                    
        else:
            print("Couldn't determine input")


        
       
if __name__ == '__main__':
    try:
        brain_action()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
