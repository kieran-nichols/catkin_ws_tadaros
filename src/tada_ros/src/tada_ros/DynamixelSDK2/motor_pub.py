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
HOMING_OFFSET2              =-1024 #need to figure out how to set !!!
POSITION_P_GAIN             =2760  #need to figure out how to set !!!

# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold Question, this is set twice !!! delete one?


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
current_angles = [0, 0]

def read_goal(which_motor):
    time.sleep(0.3)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, which_motor, ADDR_GOAL_POSITION)
    print("reading motor ", which_motor, " goal angle and counts: ", int(dxl_present_position*0.087891), " ", dxl_present_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def write_goal(which_motor, angle):
    time.sleep(0.3)
    print("writing motor ", which_motor, " goal angle and counts: ", angle, " ", int(angle/0.087891))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, which_motor, ADDR_GOAL_POSITION, int(angle/0.087891))
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
  
    
#to move a motor to an angle
def move_motor(which_motor, next_angle):
    print("\n")
    print("START MOVING MOTOR: ", which_motor)
    
    #getting current angle
    global current_angles
    current_angle = current_angles[which_motor-1]

        
    #get correct angle
    #next_angle = new_angle_calculate(current_angle, next_angle)
    print("data (current count, current angle, next count, next angle): ", int(current_angle/0.087891), current_angle, int(next_angle/0.087891), next_angle)
        
    #read
    print("Before write: ")
    read_goal(which_motor)
            
    #write
    write_goal(which_motor, next_angle)
            
    #read
    print("After write: ")
    read_goal(which_motor)
        
    print("END MOVING MOTOR: ", which_motor)
    print("\n")
 
#to calculate which angle we need to go locally to achive given angle
def new_angle_calculate(current_angle, future_angle):
    if abs(current_angle-future_angle)<=180:
        return future_angle
    else:
        print("Algo Triggered ", abs(current_angle-future_angle), current_angle, future_angle )
        current_sign2 = np.sign(current_angle-future_angle) #if pos we add to 360*x, if negetive we then subtract
        #+3 for noise and error
        #Equation to where to move
        result_angle = future_angle%180*current_sign2+math.floor((current_angle+3)/360)*360
        if result_angle<0:
            print("Error! The angle is going into 0 and will wrap around and mess up the motor location! Enter a new angle!")
            return 0
        print("Math2 (where rotating, new angle, old pos add-on): ", result_angle, future_angle%360, math.floor((current_angle+3)/360)*360)
        return result_angle

#to set-up a motor
def motor_set_up(which_motor):
    print("\n")
    print("SET-UP MOTOR ", which_motor)
    #reset while not changing id or baud
    #packetHandler.factoryReset(portHandler, which_motor, 0x02)
    set_up_operating_mode(which_motor)
    set_up_torque(which_motor)
    print("END SET-UP MOTOR ", which_motor)
    print("\n")

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
    #max reading is 377488800? and that is at 180 position
    #ERROR: does not move to 360 or 0 but does go to 1 or 359
    #Increase in degrees CCW and Decrease CW
    # full rotation if over 360 move
    input_motor_angle1 = int(MotorDataMsg.motor1_angle)
    input_motor_angle2 = int(MotorDataMsg.motor2_angle)
    #too many message sening, so only send if new dir
    
    move_motor(1, input_motor_angle1)
    move_motor(2, input_motor_angle2)

    #time.sleep(0.1)

def read_write_py_node():
    global current_angles


    #seting up the publisher
    rospy.init_node('motor_pub', anonymous=True)
    pub = rospy.Publisher('motor_listen', MotorListenMsg, queue_size=100)
    sub = rospy.Subscriber('motor_command', MotorDataMsg, set_goal_pos_callback)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        motor_msg = MotorListenMsg()
        dxl_present_position1, dxl_comm_result1, dxl_error = packetHandler.read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION)
        dxl_present_position2, dxl_comm_result2, dxl_error = packetHandler.read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION)
        motor_msg.current_angle1 = int(dxl_present_position1*0.087891)
        current_angles[0] = int(dxl_present_position1*0.087891) #in angle
        motor_msg.current_angle2 = int(dxl_present_position2*0.087891) #in angle
        current_angles[1] = int(dxl_present_position2*0.087891)#in angle
        motor_msg.motor_fail = False
        motor_msg.toff = 0
        pub.publish(motor_msg)
        rate.sleep()
    rospy.spin()

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
