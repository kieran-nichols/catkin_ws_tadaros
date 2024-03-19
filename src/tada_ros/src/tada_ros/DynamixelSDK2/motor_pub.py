#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

#*******************************************************************************
#***********************     Clear Multi Turn Example [Extended Position Control Mode]   ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Note that the XL320 does support Extended Position Control Mode
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ki Jong Gil (Gilbert)
#  Maintainer : Zerom, Will Son
# ****

from __future__ import print_function
import rospy
import os
import sys
import signal
from tada_ros.msg import KillConfirmationMsg, MotorListenMsg, MotorDataMsg
from time import sleep
from std_msgs.msg import String, Bool
import math
import numpy as np

from dynamixel_sdk import *

#to init the motor communication
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 57600 #57600     # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
EXT_POSITION_CONTROL_MODE   = 4                 # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
MAX_POSITION_VALUE          = 1048575     #currently isn't called anywhere      # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
DXL_MOVING_STATUS_THRESHOLD = 20             #CHANGE this to adjust accuracy!!!!   # Dynamixel will rotate between this value
#goal_position               = -4095
ADDR_OPERATING_MODE         = 11
POSITION_P_GAIN             =2760  #need to figure out how to set !!!
ADDR_POSITION_P_GAIN        =84
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold Question, this is set twice !!! delete one?


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
current_angles = [0, 0]
moving_angles = [0, 0]

#global true/false variable
#if true means writting/reading, if false means that we are not writting/reading.
occupying_bus = False

#ratio of counts to angle
motor_count_to_angles = 0.087891

#sleep values
sleep_reboot = 0.1
sleep_waiting_for_move = 1

#how much angle deviation do we allow
angle_error_threshold = 3


#to handle jamming reset
def handle_jamming_reboot(which_motor, angle):
    global reading_position
    global motor_count_to_angles
    
    print("Rebooting motor ", which_motor, " due to error")
    
    #reboot
    packetHandler.reboot(portHandler, which_motor)
    sleep(sleep_reboot)
    
    #setting up post reboot
    set_up_operating_mode(which_motor)
    set_up_torque(which_motor)
    set_up_position_p_gain(which_motor)
    
    #sending the command again
    write_goal(which_motor, angle)
    
#to check whether the movement was complete
def check_movement(which_motor, fix):
    global motor_count_to_angles
    global angle_error_threshold
    global current_angles
    global moving_angles
    global occupying_bus
    
    #getting values
    sleep(sleep_waiting_for_move) #delay for the expected time to move
    current_angle = current_angles[which_motor-1] #getting current angle again
    moving_angle = moving_angles[which_motor-1] #getting where we wanted to move
    
    #comparing values
    print("\nANGLE DIF: ", current_angle-moving_angle, " FOR MOTOR ", which_motor)
    if abs(current_angle-moving_angle) > angle_error_threshold:
        print("NOT MOVED ERROR MOTOR", which_motor)
        if fix:
            handle_jamming_reboot(which_motor, moving_angle)

def unsigned_to_signed_int(number):
    return int.from_bytes((number).to_bytes(4, byteorder='big', signed=False), byteorder = 'big', signed = True)
    
def read_goal(which_motor):
    global occupying_bus
    global reading_position
    global motor_count_to_angles
    
    #waiting for the bus to finish its previous read
    while occupying_bus:
        continue
    
    #taking the bus
    occupying_bus = True
    
    #reading the bus
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, which_motor, ADDR_GOAL_POSITION)
    dxl_present_position = unsigned_to_signed_int(dxl_present_position)
    print("reading motor ", which_motor, " goal angle and counts: ", int(dxl_present_position*motor_count_to_angles), " ", dxl_present_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    #releasing it
    occupying_bus = False

def write_goal(which_motor, angle):
    global occupying_bus
    global motor_count_to_angles
    
    #waiting for the bus to finish its previous read
    while occupying_bus:
        continue
    
    #taking the bus
    occupying_bus = True
    
    #writing
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, which_motor, ADDR_GOAL_POSITION, int(angle/motor_count_to_angles))
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        handle_jamming_reboot(which_motor, angle)
    
    #releasing the bus
    occupying_bus = False
  
    
#to calculate which angle we need to go locally to achive given angle
def new_angle_calculate(current_angle, future_angle):
    present=current_angle%360 #current local position in degrees (aka between 0 to 360 degrees)
    goal=future_angle%360 #this converts it into 0 to 360 so we don't have to deal with negative numbers. e.g. it would convert -90 deg to 270 deg which are equivalent for our purposes

    if (abs(goal-present)>180): # if we would move more than 180 deg we will change the motor direction
        if (goal-present)>0:#moves CW 
            temp=goal-360 #half of the rotation
            change=temp-present #the other half of the rotation
        elif (goal-present)<0:  #moves CCW
            temp=360-present  #half of the rotation
            change=temp+goal  #other half of the rotation
    else: #This is a catch, move default direction")
        change=goal-present
        
    result_angle=change+current_angle #this adds the change to the previous location to get the position we want it to move to
    return result_angle

#to read a motor's position
def read_angle_motor(which_motor):
    global current_angles
    global motor_count_to_angles
    global occupying_bus
    
    #read when available
    while occupying_bus:
        continue
    
    occupying_bus = True #started the read and taking the bus
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, which_motor, ADDR_PRESENT_POSITION) #reading motor position
    dxl_present_position = unsigned_to_signed_int(dxl_present_position) #convertig to signed int
    current_angles[which_motor-1] = int(dxl_present_position*motor_count_to_angles) #in angle
    occupying_bus = False #ended the read and releasing the bus
    
    return current_angles[which_motor-1]

#to move a motor to an angle
def move_motor(which_motor, next_angle):
    global current_angles
    global moving_angles
    
    current_angle = current_angles[which_motor-1]

    #takes the shortest path to get to the right location
    local_next_angle = new_angle_calculate(current_angle, next_angle)
    moving_angles[which_motor-1] = local_next_angle
    
    #write
    write_goal(which_motor, local_next_angle)
    
def set_up_position_p_gain(which_motor):
    print("settin up position p gain for motor ", which_motor)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, which_motor, ADDR_POSITION_P_GAIN, POSITION_P_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print("result")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("error")
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("position p gain has been set for motor ", which_motor)


#to set-up a motor
def motor_set_up(which_motor):
    print("\n")
    print("REBOOT MOTOR ", which_motor)
    packetHandler.reboot(portHandler, which_motor)
    sleep(sleep_reboot)
    print("SET-UP MOTOR ", which_motor)
    #reset while not changing id or baud
    #packetHandler.factoryReset(portHandler, which_motor, 0x02)
    set_up_operating_mode(which_motor)
    set_up_torque(which_motor)
    set_up_position_p_gain(which_motor)
    
    print("END SET-UP MOTOR ", which_motor)
    print("\n")
    #make sure motor always starts at position 0
    write_goal(which_motor, 0)

#To set-up operating mode for a motor
def set_up_operating_mode(which_motor):
    # Set operating mode to extended position control mode #if we want it to be vel control=0, pos control=3, extended pos control=4, pwm=16 change this EXT_POSI...)
    print("settin up mode for motor ", which_motor)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, which_motor, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("result")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("error")
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode has been set for motor ", which_motor)

#To set-up torque for a motor
def set_up_torque(which_motor):
    # Enable Dynamixel Torque
    print("settin up torque for motor ", which_motor)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, which_motor, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("result")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))       
        
    elif dxl_error != 0:
        print("error")
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque has been set for motor ", which_motor)


def set_goal_pos_callback(MotorDataMsg):
    input_motor_angle1 = int(MotorDataMsg.motor1_angle)
    input_motor_angle2 = int(MotorDataMsg.motor2_angle)
    
    #moving the motor
    move_motor(1, input_motor_angle1)
    move_motor(2, input_motor_angle2)

    #checking whether the motor movement was correct, we have to do it after so that we don't slow down the motor movement
    #check if moved correctly
    check_movement(1, False)
    check_movement(2, False)
    

def read_write_py_node():
    global occupying_bus
    global moving_angles
    global current_angles

    #seting up the publisher
    rospy.init_node('motor_pub', anonymous=True)
    pub = rospy.Publisher('motor_listen', MotorListenMsg, queue_size=100)
    sub = rospy.Subscriber('motor_command', MotorDataMsg, set_goal_pos_callback)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        motor_msg = MotorListenMsg() # creating message instance
        #skipping if the bus is busy
        if occupying_bus:
            continue
        #getting angle and publishing
        motor_msg.current_angle1 = read_angle_motor(1)
        motor_msg.current_angle2 = read_angle_motor(2)
        motor1_moved = abs(current_angles[0]-moving_angles[0]) > angle_error_threshold
        motor2_moved = abs(current_angles[0]-moving_angles[0]) > angle_error_threshold
        motor_msg.motor1_fail = motor1_moved
        motor_msg.motor2_fail = motor2_moved
        motor_msg.toff = 0
        pub.publish(motor_msg)
        rate.sleep()
       
#main function
def main():
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    
    
    #Setting up motors
    motor_set_up(1)
    motor_set_up(2)
    
    #start the node
    print("Ready to get & set Position.")
    read_write_py_node()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
