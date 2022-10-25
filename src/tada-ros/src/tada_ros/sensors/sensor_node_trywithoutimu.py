#!/usr/bin/env python
import rospy
import os
import sys
import signal
from tada_ros.msg import KillConfirmationMsg, IMUDataMsg, ReconDataMsg
from std_msgs.msg import Bool
from time import sleep
from std_msgs.msg import String
#from tada_ros.sensors import IMU_controller


import smbus # SMBus module of I2C
import time
import os, sys
import math
from time import sleep
from enum import Enum
from tada_ros.msg import IMUDataMsg, ReconDataMsg
from tada_ros.global_info import constants

import numpy as np
import subprocess
import csv
DEBUG_FLAG = 0
linear_correction = 1

import subprocess
import csv

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
class SensorNode():
    bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
    Device_Address = 0x68   # MPU6050 device address
    m1_pos = 0
    m2_pos = 0
    # Initial variables
    state = 0
    start = time.time()
    start_time = time.time()
    initial_itr = 0
    swing_time = 0
    swing = [0, 0, 0]
    avg_swing = [0.7, 0.7, 0.7]
    avg_val_swing = 0
    initial_itr1 = 0
    gyro_thres = 5
    accel_thres = 0.5
    state=0
    swing_time=0


    pub_killed_confirm = None
            # killing the program is its own topic so that there's minimal delay

    def __init__(self):
        
        #rospy.Subscriber('kill_all_topic', Bool, handle_kill_command)
        print("initialized IN SENSOR")
        #MPU_Init()
        pub_sensing = rospy.Publisher('sensing_topic', String, queue_size=10)
        rospy.init_node('sensor_node', anonymous=True)
        rate = rospy.Rate(100)
        imu = IMU_controller.IMUController()
        while not rospy.is_shutdown():
            imu_data = imu.get_data()
            
            msg_imu = imu_data.to_string()
            pub_sensing.publish(msg_imu)
            
            rate.sleep()
        
        
        
        # queue_size argument limits the number of queued messages if any
        # subscriber is not receiving them fast enough.
        
    def talker():
        #pub_killed_confirm = rospy.Publisher('kill_confirmation_topic', Bool, queue_size=10)
        
        #pub_recon = rospy.Publisher('recon_topic', ReconDataMsg, queue_size=10)
        # anonymous=True ensures that the name is unique by adding random numbers
        rate = rospy.Rate(100) # 100hz
        #
        # initialize the IMU controller
        #imu = IMU_controller.IMUController()
        while not rospy.is_shutdown():
                # get sensor input; uncomment bias if you want live bias data
                #bias = imu.get_bias()
                #imu_data = imu.get_data(bias)

                # do the IMU reconstruction
                #recon_data = imu.imu_reconstruction(imu_data,bias)

                # send sensor input to topic
                imu_string  = read_cooked_data()
                #print(imu_string)
                pub_sensing.publish(imu_string)

                # send recon output to topic
                #msg_recon = recon_data.to_ROS_message()
                #pub_recon.publish(msg_recon)
                #rate.sleep() # sleeps to maintain selected rate
                #rospy.spin()
    def handle_kill_command( data):
        # for debugging
        print("")
        rospy.loginfo(rospy.get_caller_id() + " I heard kill boolean: %s", data.data)
        print("")
        if data.data == True:
            kill_sensors()

        # send confirmation message
        msg = KillConfirmationMsg(motors_killed=True, sensors_killed=False)
        pub_killed_confirm.publish(msg)

        # interrupt the program like a Keyboard Interrupt
        os.kill(os.getpid(), signal.SIGTERM)
            
    def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)
            
    def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        
        #concatenate higher and lower value
        value = ((high << 8) | low)
            
        #to get signed value from mpu6050
        if(value > 32768):
              value = value - 65536
        return value
        
    def read_cooked_data():
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        imu_string  = str(Ax) + " " + str(Ay) + " " + str(Az) + " " +str(Gx) + " " +str(Gy) + " " +str(Gz)
        return imu_string
        
   
    def kill_sensors():
        # TODO: sensor-specific code
        print("Killing Sensor Node")
if __name__ == '__main__':
    SensorNode()
#def Publishing_Sensor(void):
   # print("in sensor")
   # return SensorNode.talker()
