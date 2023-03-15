#!/usr/bin/env python

# Much of this is from Interfacing Raspberry Pi with MPU6050
# http://www.electronicwings.com

import smbus # SMBus module of I2C
import numpy
import time
import os, sys
import math
import numpy as np
from time import sleep
from enum import Enum
from tada_ros.msg import IMUDataMsg, ReconDataMsg
from tada_ros.global_info import constants


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
    bus = smbus.SMBus(1)
    cur_time = time.time()
    measured_dt = constants.DT
    Device_Address = 0x68   # MPU6050 device address
    ## initialization of IMU
    def __init__(self):
        # initialize the MPU6050 Module IMU
        self.bus.write_byte_data(DEVICE_ADDR, SMPLRT_DIV, 7)
        self.bus.write_byte_data(DEVICE_ADDR, PWR_MGMT_1, 1)
        self.bus.write_byte_data(DEVICE_ADDR, CONFIG, 0)
        self.bus.write_byte_data(DEVICE_ADDR, GYRO_CONFIG, 24)
        self.bus.write_byte_data(DEVICE_ADDR, INT_ENABLE, 1)
        
        self.state = 0
        self.start = time.time()
        self.start_time = time.time()
        self.initial_itr = 0
        self.swing_time = 0
        self.swing = [0, 0, 0]
        self.avg_swing = [0.3, 0.3, 0.3]
        self.avg_val_swing = 0
        self.initial_itr1 = 0
        self.gyro_thres = 60 #FINE TUNE THIS
        self.accel_thres = 0.5
        self.swing_time=0

    ## read raw bytes from IMU
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(DEVICE_ADDR, addr)
        low = self.bus.read_byte_data(DEVICE_ADDR, addr+1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
    
    ## collect continuous steam of IMU accel and gyro data and output de-biased data
    def get_data(self):
        
        # Read Accelerometer values
        raw_accel_x = self.read_raw_data(ACCEL_XOUT_H)
        raw_accel_y = self.read_raw_data(ACCEL_YOUT_H)
        raw_accel_z = self.read_raw_data(ACCEL_ZOUT_H)

        # Read gyro_yroscope values
        raw_gyro_x = self.read_raw_data(GYRO_XOUT_H)
        raw_gyro_y = self.read_raw_data(GYRO_YOUT_H)
        raw_gyro_z = self.read_raw_data(GYRO_ZOUT_H)

        # Adjust raw data for scale factor
        accel_x = raw_accel_x/16384.0
        accel_y = raw_accel_y/16384.0
        accel_z = raw_accel_z/16384.0

        gyro_x = raw_gyro_x/131.0
        gyro_y = raw_gyro_y/131.0
        gyro_z = raw_gyro_z/131.0

        #SWING
        if gyro_z > self.gyro_thres: # swing
            # ~ initial_itr = 0
            self.state = 1
            # collect the swing time and save the data only once
            if self.initial_itr == 0:
                
                # ~ print(self.swing_time)
                self.start_time = time.time()
                self.swing.append(self.swing_time)
                avg_swing = self.swing[-3:]
                self.avg_val_swing = np.mean(avg_swing)
                # limit the avg swing time to be at minimum 0.2
                if self.avg_val_swing < 0.2: self.avg_val_swing = 0.2
                print(self.avg_val_swing)
                # ~ self.swing_time = 0
                self.initial_itr = 1
                self.initial_itr1 = 0
            else: # to ensure that swing is only appended once
                self.swing_time = time.time() - self.start_time
        # when not saving data, move the motor at the first itr
        # and collect the swing time
        else: # stance
            # ~ if (self.initial_itr1==0):
                # ~ self.avg_swing_command = int(1000 * self.avg_val_swing)
    #             move_command(avg_swing_command)
                # sleep(avg_val_swing+0.3)
                self.initial_itr1 = 1
                self.initial_itr = 0
            # ~ else:
                self.state = 0
                # ~ sleep(0.5)
        
        #SWING
        current_time = rospy.Time.now()
        current_time_value = current_time.to_sec()
        imu_data = IMUData(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, self.state, self.avg_val_swing, current_time_value)
    
        return imu_data
