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
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
old_goal = 0

def set_goal_pos_callback(MotorDataMsg):
    global old_goal
    moving_counts = int(MotorDataMsg.motor1_move/0.087891)
    #too many message sening, so only send if new dir
    if old_goal != moving_counts:
        print("Set Goal Position  %s = %s" % (moving_counts, MotorDataMsg.motor1_move))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, moving_counts)
        old_goal = moving_counts
    #time.sleep(0.1)
        
def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    #print("Present Position of ID %s = %s" % (DXL_ID, dxl_present_position))
    return dxl_present_position

def read_write_py_node():
    #rospy.Service('get_position', GetPosition, get_present_pos)

    #seting up the publisher
    rospy.init_node('motor_pub', anonymous=True)
    pub = rospy.Publisher('motor_listen', MotorListenMsg, queue_size=100)
    sub = rospy.Subscriber('motor_command', MotorDataMsg, set_goal_pos_callback)
    #rospy.Service('get_position', GetPosition, get_present_pos)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        motor_msg = MotorListenMsg()
        motor_msg.curr_pos2 = 0
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        motor_msg.curr_pos1 = dxl_present_position*0.087891
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

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    
    #dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, int(180/0.087891))
    print("Ready to get & set Position.")
    read_write_py_node()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass