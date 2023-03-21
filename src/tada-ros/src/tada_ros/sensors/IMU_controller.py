#!/usr/bin/env python

# Much of this is from Interfacing Raspberry Pi with MPU6050
# http://www.electronicwings.com

import smbus # SMBus module of I2C
from mpu6050 import mpu6050
import numpy
import time
import os, sys
import rospy
import math
import numpy as np
from time import sleep
from enum import Enum
from tada_ros.msg import IMUDataMsg, ReconDataMsg
from tada_ros.global_info import constants
import rospy


DEBUG_FLAG = 0
linear_correction = 1

# todo: what of this should be in constants file?
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

DEVICE_ADDR = 0x68   # MPU6050 device address

class Triple():
    def __init__(self, x, y, z):
        # casting to float to avoid integer division
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class IMUData():
    def __init__(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, state, swing_time, t):
        self.accel = Triple(accel_x, accel_y, accel_z)
        self.gyro = Triple(gyro_x, gyro_y, gyro_z)
        self.swing = Triple(state, swing_time, t)

    def accel_magnitude(self):
        return numpy.linalg.norm([self.accel.x, self.accel.y, self.accel.z])

    def gyro_magnitude(self):
        return numpy.linalg.norm([self.gyro.x, self.gyro.y, self.gyro.z])

    def to_ROS_message(self):
        return IMUDataMsg(self.accel.x, self.accel.y, self.accel.z, \
                self.gyro.x, self.gyro.y, self.gyro.z, self.swing.x, self.swing.y , self.swing.z)

    def to_string(self):
        str = "accel_x: %.6f; accel_y: %.6f; accel_z: %.6f;\n" \
            % (self.accel.x, self.accel.y, self.accel.z)
        str += "gyro_x: %.6f; gyro_y: %.6f; gyro_z: %.6f;\n" \
            % (self.gyro.x, self.gyro.y, self.gyro.z)
        str += "swing state: %.6f; time: %.6f; " \
            % (self.swing.x, self.swing.y, self.swing.z)
        return str

    def print(self):
        print("accel_x: %.6f; accel_y: %.6f; accel_z: %.6f; " \
            % (self.accel.x, self.accel.y, self.accel.z))
        print("gyro_x: %.6f; gyro_y: %.6f; gyro_z: %.6f; " \
            % (self.gyro.x, self.gyro.y, self.gyro.z))
        print ("swing state: %.6f; time: %.6f; " \
            % (self.swing.x, self.swing.y, self.swing.z))

def ROS_message_to_IMUData(msg_data):
    #print("IMU message function")
    return IMUData(msg_data.accel_x, msg_data.accel_y, msg_data.accel_z, \
                    msg_data.gyro_x, msg_data.gyro_y, msg_data.gyro_z, msg_data.state, msg_data.swing_time, msg_data.t)

class IMUController():
    # initialize class variables
    # bus for I2C
    cur_time = time.time()
    measured_dt = constants.DT
    Device_Address = 0x68   # MPU6050 device address
    sensor = mpu6050(Device_Address)
    ## initialization of IMU
    def __init__(self):

        self.state = 0
        self.start = time.time()
        self.start_time = time.time()
        self.initial_itr = 0
        self.swing_time = 0
        self.swing = [0, 0, 0]
        self.avg_swing = [0.3, 0.3, 0.3]
        self.avg_val_swing = 0
        self.gyro_thres = 20 #FINE TUNE THIS
        self.accel_thres = 0.5
        self.swing_time=0

    ## collect continuous steam of IMU accel and gyro data and output de-biased data
    def get_data(self):
        
        # Adjust raw data for scale factor
        accel_x = self.sensor.get_accel_data().get('x')
        accel_y = self.sensor.get_accel_data().get('y')
        accel_z = self.sensor.get_accel_data().get('z')

        gyro_x = self.sensor.get_gyro_data().get('x')
        gyro_y = self.sensor.get_gyro_data().get('y')
        gyro_z = self.sensor.get_gyro_data().get('z')

        #SWING
        if gyro_z > self.gyro_thres: # swing
            self.state = 1
            # collect the swing time and save the data only once
            if self.initial_itr == 0:
                self.start_time = time.time()
                #print(self.avg_val_swing)
                self.initial_itr = 1
        # when not saving data, move the motor at the first itr
        # and collect the swing time
        else: # stance
            if self.initial_itr ==1:
                
                self.swing_time = time.time() - self.start_time
                if self.swing_time > 0.1:
                    self.swing.append(self.swing_time)
                    avg_swing = self.swing[-3:]
                    self.avg_val_swing = np.mean(avg_swing)
                # limit the avg swing time to be at minimum 0.2
                
            self.initial_itr = 0
            self.state = 0
        
        current_time = rospy.Time.now()
        current_time_value = current_time.to_sec()
        current_time_value = current_time_value%100000
        imu_data = IMUData(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, self.state, self.avg_val_swing, current_time_value)
    
        return imu_data
