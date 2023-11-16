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
#from dynamixel_sdk_examples.srv import *
#from dynamixel_sdk_examples.msg import *

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
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
EXT_POSITION_CONTROL_MODE   = 4                 # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
MAX_POSITION_VALUE          = 1048575     #currently isn't called anywhere      # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
DXL_MOVING_STATUS_THRESHOLD = 20             #CHANGE this to adjust accuracy!!!!   # Dynamixel will rotate between this value
#goal_position               = -4095
ADDR_OPERATING_MODE         = 11

# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
curr_pos1 = 0
old_pos1 = 10000000000000000000000
curr_pos2 = 0
old_pos2 = 10000000000000000000000
def set_goal_pos_callback(MotorDataMsg):
    global curr_pos1
    global old_pos1
    global curr_pos2
    global old_pos2
    #ERROR: does not move to 360 or 0 but does go to 1 or 359
    #Increase in degrees CCW and Decrease CW
    # full rotation if over 360 move
    moving_counts2 = int(MotorDataMsg.motor2_move)
    #too many message sening, so only send if new dir
    if old_pos2 != moving_counts2:
        if abs(curr_pos2-moving_counts2)<=180:
            input_motor_rotation2= moving_counts2
        else:
            cur_sign2 = np.sign(curr_pos2-moving_counts2) #if pos we add to 360*x, if negetive we then subtract
            print("Algo Triggered ", abs(curr_pos2-moving_counts2), curr_pos2, moving_counts2 )
            #+5 for noise and error
            input_motor_rotation2 = moving_counts2%180*cur_sign2+math.floor((curr_pos2+5)/360)*360
            if input_motor_rotation2<0:
                print("Error! The angle is going into 0 and will wrap around and mess up the motor location! Enter a new angle!")
                #return
            print("Math: ", input_motor_rotation2, moving_counts2%360, math.floor((curr_pos2+3)/360)*360)
        print("Set Goal Position  %s = %s from %s" % (input_motor_rotation2, MotorDataMsg.motor2_move, curr_pos2))
        #we rransfor from angle into cunts right before wrighting
        dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(input_motor_rotation2/0.087891))
        old_pos2 = moving_counts2
    #motor11
    moving_counts1 = int(MotorDataMsg.motor1_move)
    #too many message sening, so only send if new dir
    if old_pos1 != moving_counts1:
        if abs(curr_pos1-moving_counts1)<=180:
            input_motor_rotation1= moving_counts1
        else:
            cur_sign1 = np.sign(curr_pos1-moving_counts1) #if pos we add to 360*x, if negetive we then subtract
            print("Algo Triggered ", abs(curr_pos1-moving_counts1), curr_pos1, moving_counts1 )
            #+5 for noise and error
            input_motor_rotation1 = moving_counts1%180*cur_sign1+math.floor((curr_pos1+5)/360)*360
            if input_motor_rotation1<0:
                print("Error! The angle is going into 0 and will wrap around and mess up the motor location! Enter a new angle!")
                #return
            print("Math: ", input_motor_rotation1, moving_counts1%360, math.floor((curr_pos1+3)/360)*360)
        print("Set Goal Position  %s = %s from %s" % (input_motor_rotation1, MotorDataMsg.motor1_move, curr_pos1))
        #we rransfor from angle into cunts right before wrighting
        dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(input_motor_rotation1/0.087891))
        old_pos1 = moving_counts1
    
    #motor2
   
    #time.sleep(0.1)

def read_write_py_node():
    global curr_pos1
    global curr_pos2


    #seting up the publisher
    rospy.init_node('motor_pub', anonymous=True)
    pub = rospy.Publisher('motor_listen', MotorListenMsg, queue_size=100)
    sub = rospy.Subscriber('motor_command', MotorDataMsg, set_goal_pos_callback)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        motor_msg = MotorListenMsg()
        dxl_present_position1, dxl_comm_result1, dxl_error = packetHandler.read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION)
        dxl_present_position2, dxl_comm_result2, dxl_error = packetHandler.read4ByteTxRx(portHandler, 2, ADDR_PRESENT_POSITION)
        motor_msg.curr_pos1 = int(dxl_present_position1*0.087891)
        curr_pos1 = int(dxl_present_position1*0.087891)
        motor_msg.curr_pos2 = int(dxl_present_position2*0.087891)
        curr_pos2 = int(dxl_present_position2*0.087891)
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
    
        
    # Set operating mode to extended position control mode #if we want it to be vel control=0, pos control=3, extended pos control=4, pwm=16 change this EXT_POSI...)
    dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
    dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
    if dxl_comm_result1 != COMM_SUCCESS or dxl_comm_result2 != COMM_SUCCESS:
        print("error 1")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("error 2")
        print("%s" % packetHandler.getRxPacketError(dxl_error1))
    else:
        print("Operating mode changed to extended position control mode.")
    
    # Enable Dynamixel Torque
    dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result1 != COMM_SUCCESS or dxl_comm_result2 != COMM_SUCCESS:
        print("error1")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error1 != 0 or dxl_error2 != 0:
        print("error2")
        print("%s" % packetHandler.getRxPacketError(dxl_error1))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    print("Ready to get & set Position.")
    read_write_py_node()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
