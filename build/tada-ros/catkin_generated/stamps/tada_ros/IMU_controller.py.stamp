#!/usr/bin/env python

# Much of this is from Interfacing Raspberry Pi with MPU6050
# http://www.electronicwings.com

import smbus # SMBus module of I2C
import numpy
import time
import os, sys
import math
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

class WalkStatus(Enum):
    STANCE = 0
    SWING = 1

class KalmanFilter():
    def __init__(self):
        # casting to float to avoid integer division
        self.pkf = float(0)
        self.qkf = float(1.5e-5 * constants.DT)
        self.rkf = float(1.5e-1 * constants.DT)
        self.k = float(0)

class Triple():
    def __init__(self, x, y, z):
        # casting to float to avoid integer division
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class IMUData():
    def __init__(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        self.accel = Triple(accel_x, accel_y, accel_z)
        self.gyro = Triple(gyro_x, gyro_y, gyro_z)

    def accel_magnitude(self):
        return numpy.linalg.norm([self.accel.x, self.accel.y, self.accel.z])

    def gyro_magnitude(self):
        return numpy.linalg.norm([self.gyro.x, self.gyro.y, self.gyro.z])

    def to_ROS_message(self):
        return IMUDataMsg(self.accel.x, self.accel.y, self.accel.z, \
                self.gyro.x, self.gyro.y, self.gyro.z)

    def to_string(self):
        str = "accel_x: %.6f; accel_y: %.6f; accel_z: %.6f;\n" \
            % (self.accel.x, self.accel.y, self.accel.z)
        str += "gyro_x: %.6f; gyro_y: %.6f; gyro_z: %.6f" \
            % (self.gyro.x, self.gyro.y, self.gyro.z)
        return str

    def print(self):
        print("accel_x: %.6f; accel_y: %.6f; accel_z: %.6f; " \
            % (self.accel.x, self.accel.y, self.accel.z))
        print("gyro_x: %.6f; gyro_y: %.6f; gyro_z: %.6f" \
            % (self.gyro.x, self.gyro.y, self.gyro.z))

def ROS_message_to_IMUData(msg_data):
    return IMUData(msg_data.accel_x, msg_data.accel_y, msg_data.accel_z, \
                    msg_data.gyro_x, msg_data.gyro_y, msg_data.gyro_z)

class ReconData():
    def __init__(self, timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, \
                accel_x, accel_y, accel_z):
        self.timestamp = timestamp
        self.pos = Triple(pos_x, pos_y, pos_z)
        self.vel = Triple(vel_x, vel_y, vel_z)
        self.accel = Triple(accel_x, accel_y, accel_z)

    def to_ROS_message(self):
        return ReconDataMsg(self.timestamp, \
                            self.pos.x, self.pos.y, self.pos.z, \
                            self.vel.x, self.vel.y, self.vel.z, \
                            self.accel.x, self.accel.y, self.accel.z)

    def ROS_message_to_ReconData(msg_data):
        return ReconData(msg_data.timestamp, \
                        msg_data.pos_x, msg_data.pos_y, msg_data.pos_z, \
                        msg_data.vel_x, msg_data.vel_y, msg_data.vel_z, \
                        msg_data.accel_x, msg_data.accel_y, msg_data.accel_z)
## class deidcated to IMU data collection and processing

class IMUController():
    # initialize class variables
    # bus for I2C
    bus = smbus.SMBus(1)
    cur_time = time.time()
    measured_dt = constants.DT
    rel_path = "reconstruction_files/reconstruction_file_%s.txt" % \
        time.strftime("%Y-%m-%d_%H-%M-%S")
    file_path = os.path.join(os.path.dirname(__file__), "../", rel_path)
    reconstruction_file = open(file_path,"w+")
    ## initialization of IMU
    def __init__(self):
        # initialize the MPU6050 Module IMU
        self.bus.write_byte_data(DEVICE_ADDR, SMPLRT_DIV, 7)
        self.bus.write_byte_data(DEVICE_ADDR, PWR_MGMT_1, 1)
        self.bus.write_byte_data(DEVICE_ADDR, CONFIG, 0)
        self.bus.write_byte_data(DEVICE_ADDR, GYRO_CONFIG, 24)
        self.bus.write_byte_data(DEVICE_ADDR, INT_ENABLE, 1)

        # boolean for whether the IMU was most recently measured as in motion or not
        self.is_low_motion = True
        # tracks the number of seconds we measured IMU to be in low/non-low motion
        # in a row
        self.low_motion_seconds = 0
        self.nonlow_motion_seconds = 0
        # keeps track of the amount of consecutive time spent in stance or swing
        # and the previous consecutive amount of time spent in swing
        self.stance_time = 0
        self.swing_time = 0
        self.prev_swing_time = 0
        # time between start of swing to start of next swing
        self.stride_time = 0
        # keeps track of current velocity and position (stride_length is length
        # of stride and p_speed is loc of last stride start)
        self.vel = [float(0)] * 3
        self.pos = [float(0)] * 3
        #  position of the beginning of the previous stride
        self.prev_pos = [float(0)] * 3
        self.walking_speed = 0
        # boolean for whether the speed has already been estimated for the
        # current stride
        self.stride_speed_estimated = False
        # keeps track of if we're currently in stance or swing
        self.walk_status = WalkStatus.STANCE
        # empty Kalman Filter and quaternion
        self.kalman = KalmanFilter()
        self.quaternion = [float(0)] * 4
        # boolean for whether we have performed imu_integrate yet
        self.is_first_integration = True

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
    
    ## collect raw values of IMU for bias correction
    def get_bias(self):
        # Read Accelerometer values
        bias_accel_x = self.read_raw_data(ACCEL_XOUT_H)
        bias_accel_y = self.read_raw_data(ACCEL_YOUT_H)
        bias_accel_z = self.read_raw_data(ACCEL_ZOUT_H)

        # Read gyroscope values
        bias_gyro_x = self.read_raw_data(GYRO_XOUT_H)
        bias_gyro_y = self.read_raw_data(GYRO_YOUT_H)
        bias_gyro_z = self.read_raw_data(GYRO_ZOUT_H)

        #bias_data = IMUData(4884/2, 1266/2, 567,bias_gyro_x,bias_gyro_y,bias_gyro_z) # from bias test excel
        bias_data = IMUData(bias_accel_x, bias_accel_y, -bias_accel_z+16384, bias_gyro_x, bias_gyro_y, bias_gyro_z)

        return bias_data
    
    ## collect continuous steam of IMU accel and gyro data and output de-biased data
    def get_data(self,bias):
        # Read Accelerometer values
        raw_accel_x = self.read_raw_data(ACCEL_XOUT_H)
        raw_accel_y = self.read_raw_data(ACCEL_YOUT_H)
        raw_accel_z = self.read_raw_data(ACCEL_ZOUT_H)

        # Read gyro_yroscope values
        raw_gyro_x = self.read_raw_data(GYRO_XOUT_H)
        raw_gyro_y = self.read_raw_data(GYRO_YOUT_H)
        raw_gyro_z = self.read_raw_data(GYRO_ZOUT_H)

        # Adjust raw data for scale factor
        accel_x = -(raw_accel_x - bias.accel.x)  * constants.ACCEL_SCALE_FACTOR # accel will be in yaml file; symmetry about 0
        accel_y = -(raw_accel_y + bias.accel.y) * constants.ACCEL_SCALE_FACTOR
        accel_z = (raw_accel_z + bias.accel.z) * constants.ACCEL_SCALE_FACTOR 

        gyro_x = (raw_gyro_x - bias.gyro.x) * constants.GYRO_SCALE_FACTOR   # gyros will be collected in trial before where the person stand static
        gyro_y = (raw_gyro_y - bias.gyro.y) * constants.GYRO_SCALE_FACTOR  # still need to change how gyro biases
        gyro_z = (raw_gyro_z - bias.gyro.z) * constants.GYRO_SCALE_FACTOR 

        imu_data = IMUData(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

        return imu_data

    ## reconstruct real world accel, vel, position
    def imu_reconstruction(self, imu_data,bias): # imu_data is of the type IMUData
        # get measurement for actual time since last updated
        new_time = time.time()
        self.measured_dt = new_time - self.cur_time
        self.cur_time = new_time
        if DEBUG_FLAG:
            print("measured dt is %s, we expected around %s \n" % \
                (self.measured_dt, constants.DT))

        # record position at last round
        last_pos = [float(0)] * 3
        for i in range(len(self.pos)):
            last_pos[i] = self.pos[i]

        # Initialize Kalman filter and quaternion
        if self.is_first_integration:
            # have to wait to call this until we have an initial acceleration reading
            self.initQuaternion(imu_data)

        # decide if we're in low motion or nonlow motion
        self.is_low_motion = \
            imu_data.gyro_magnitude() < constants.GYRO_THRESHOLD_VELOCITY and \
            numpy.abs(imu_data.accel_magnitude() - constants.GRAVITY) < \
            constants.ACCEL_THRESHOLD
        if self.is_low_motion:
            if DEBUG_FLAG:
                print("I'm low motion right now\n")
            self.low_motion_seconds += self.measured_dt
            self.nonlow_motion_seconds = 0
        else:
            if DEBUG_FLAG:
                print("I'm nonlow motion right now\n")
            self.nonlow_motion_seconds += self.measured_dt
            self.low_motion_seconds = 0

        # determine if we're in stance or swing
        if self.nonlow_motion_seconds >= constants.NONLOW_MOTION_THRESHOLD_SECONDS:
            self.walk_status = WalkStatus.SWING
            if DEBUG_FLAG:
                print("I'm in SWING right now\n")
        elif self.low_motion_seconds >= constants.LOW_MOTION_THRESHOLD_SECONDS:
            self.walk_status = WalkStatus.STANCE
            if DEBUG_FLAG:
                print("I'm STANCE right now\n")

        # update swing time and stance time
        if self.walk_status == WalkStatus.SWING:
            self.swing_time += self.measured_dt
            self.stance_time = 0
        else:
            if self.stance_time == 0:    # just moved to stance
                self.prev_swing_time = self.swing_time

            self.stance_time += self.measured_dt
            self.swing_time = 0

#         if self.walk_status == WalkStatus.STANCE:
#             imu_data = IMUData(imu_data.accel.x,imu_data.accel.y,imu_data.accel.z,0,0,0)
            
        # update Kalman filter and quaternion
        self.updateQuaternionKalman(imu_data)

        # calculate rotation matrix
        rm = self.quaternionToRotationMatrix()

        # calculate the real-world acceleration
        rm = [[float(i) for i in row] for row in rm]  # casting to float so no integer division
        rm_accel = numpy.inner(rm, [imu_data.accel.x, # confirm if this formulation is correct
                                    imu_data.accel.y, # to check we could tilting from horizontal to vertical; +g in world z
                                    imu_data.accel.z])
        
        # calculate velocity
        self.vel[0] += rm_accel[0] * self.measured_dt
        self.vel[1] += rm_accel[1] * self.measured_dt
        self.vel[2] += (rm_accel[2] - constants.GRAVITY) * self.measured_dt # try to make positive to keep a standard equation

        # calculate position
        self.pos = [(self.pos[i] + self.vel[i] * self.measured_dt) for i in range(len(self.pos))]
        self.pos[0] += .5 * rm_accel[0] * pow(self.measured_dt, 2)
        self.pos[1] += .5 * rm_accel[1] * pow(self.measured_dt, 2)
        self.pos[2] += .5 * (rm_accel[2] - constants.GRAVITY) * pow(self.measured_dt, 2)

        # set vel to 0 and pos to previous stance position (or have linear
        # correction when walking is in stance
        if self.walk_status == WalkStatus.STANCE:
            self.stride_speed_estimated = 0
            # assume velocity is zero in stance
            self.vel = [(0) for v in self.vel]

            if linear_correction == 1:
                # # linear velocity correction # use this code snippet
                # # Ryan: changed this to revert vel to 0 and pos to the last pos
                if numpy.abs(self.stance_time - self.measured_dt) < constants.DIFF_TIME_THRESHOLD and \
                     self.prev_swing_time != 0:
                     accel_error = [(self.vel[i] / self.prev_swing_time) \
                                     for i in range(3)]
                     self.pos = [(self.pos[i] + .5 * accel_error[i] * \
                                  pow(self.prev_swing_time, 2)) for i in range(len(self.pos))]
                else:
                    # assume no position change in stance
                    for i in range(len(self.pos)):
                        self.pos[i] = last_pos[i]

        else: # in swing
            # estimate walking speed
            # Update walking speed as soon as simultaneous speed exceed threshold
    		# and only when the previous swing phase lasts long enough
            # only update walking speed once at the beginning of a stride
            if numpy.linalg.norm(self.vel) > constants.VELOCITY_THRESHOLD and \
               self.stride_time > 0 and not self.stride_speed_estimated and \
               self.prev_swing_time > constants.PREV_SWING_TIME_THRESHOLD:
                stride_length = [(self.pos[i] - self.prev_pos[i]) for i in range(len(self.pos))]
                self.walking_speed = numpy.linalg.norm(stride_length) / self.stride_time

                # reset variables
                self.stride_time = 0
                for i in range(len(self.pos)):
                    self.prev_pos[i] = self.pos[i]
                self.stride_speed_estimated = True

        self.is_first_integration = False

        # whether in stance or swing, update stride time
        self.stride_time += self.measured_dt
        
        # reconstruction file update and printing 
        if DEBUG_FLAG == 0:
            # recon data
            print_accel = [""] * 3
            print_vel = [""] * 3
            print_pos = [""] * 3
            for i in range(3):
                 print_accel[i] = "%5.2f" % rm_accel[i]
                 print_vel[i] = "%5.2f" % self.vel[i]
                 print_pos[i] = "%5.2f" % self.pos[i]
            print_raw_accel = ["%5.3f"%imu_data.accel.x, "%5.3f"%imu_data.accel.y, "%5.3f"%imu_data.accel.z]
            self.reconstruction_file.write \
                ("Timestamp: %s, Acceleration: %s, Velocity: %s, Position: %s \n" % \
                    (self.cur_time, print_accel, print_vel, print_pos))

            print("Timestamp: %s \nRaw_Accel: %s \nAcceleration: %s \nVelocity: %s \nPosition: %s \n" % \
                    (self.cur_time, print_raw_accel, print_accel, print_vel, print_pos))
            
        else:
            # only print raw accel (x,y,z)
            self.reconstruction_file.write("Acceleration %5.3f, %5.3f, %5.3f\n" % (bias.accel.x,bias.accel.y,bias.accel.z))
            print("Acceleration %5.3f, %5.3f, %5.3f\n" % (bias.accel.x,bias.accel.y,bias.accel.z))


        recon_data = ReconData(self.cur_time, \
                                self.pos[0],self.pos[1],self.pos[2], \
                                self.vel[0], self.vel[1], self.vel[2], \
                                rm_accel[0], rm_accel[1], rm_accel[2])
#
        return recon_data

    # update the quaternion (orientation of body) using a kalman filter
    def updateQuaternionKalman(self, imu_data):
        
        # update Kalman filter
        self.kalman.pkf = self.kalman.pkf + self.kalman.qkf

        # Quaternion integration; propogating quat forward in time with measured ang vel
        # compute state transition matrix
        tiarm  = imu_data.gyro_magnitude() * self.measured_dt # total integrated angular rate magnitude
        omega = [[0, -imu_data.gyro.x, -imu_data.gyro.y, -imu_data.gyro.z],
                 [imu_data.gyro.x, 0, imu_data.gyro.z, -imu_data.gyro.y],
                 [imu_data.gyro.y, -imu_data.gyro.z, 0, imu_data.gyro.x],
                 [imu_data.gyro.z, imu_data.gyro.y, -imu_data.gyro.x, 0]]
        omega = [[o * (self.measured_dt / 2.0) for o in row] for row in omega]
        if tiarm == 0: # this is hacky but avoids divide by 0
            tiarm = 0.0000000000000000000000000001
        k1 = numpy.cos(tiarm/2.0)
        k2 = 2.0/tiarm * numpy.sin(tiarm/2.0)
        stm = [[k2 * o for o in row] for row in omega]  # state transition matrix
        for i in range(4):
            stm[i][i] = k1
        self.quaternion = numpy.inner(stm, self.quaternion) # matrix multiplication

        # apply correction if in low motion; one other way is that instead we could figure out the kalman gain and
        # we have estimated orientation matrix 
        if imu_data.gyro_magnitude() < constants.GYRO_THRESHOLD_ORIENTATION and \
            numpy.abs(imu_data.accel_magnitude() - constants.GRAVITY) \
            < constants.ACCEL_THRESHOLD:

            self.kalman.k = float(self.kalman.pkf) / \
                float(self.kalman.pkf + self.kalman.rkf)

            theta = imu_data.accel.x / constants.GRAVITY
            if theta >= -1 and theta <= 1:
                theta = numpy.arcsin(theta)
            elif theta < -1:
                theta = -numpy.pi / 2
            elif theta > 1:
                theta = numpy.pi / 2

            phi = imu_data.accel.y / numpy.cos(theta) / constants.GRAVITY
            if phi >= -1 and phi <= 1:
                phi = -numpy.arcsin(phi)
            elif phi < -1:
                phi = numpy.pi / 2
            elif phi > 1:
                phi = -numpy.pi / 2

            eul = self.quaternionToEulerian()
            new_eul = [float(0)] * 3
            new_eul[0] = eul[0]-(eul[0]-phi)*self.kalman.k
            new_eul[1] = eul[1]-(eul[1]-theta)*self.kalman.k
            new_eul[2] = eul[2]
            self.eulerianToQuaternion(new_eul)

            self.kalman.pkf = (1 - self.kalman.k) * self.kalman.pkf * \
                (1-self.kalman.k)+self.kalman.k * self.kalman.rkf * \
                self.kalman.k

    # convert the quartenion (3,1) to rotation matrix (3,3)
    def quaternionToRotationMatrix(self):
        q = self.quaternion
        # quaterion from vsf code
#         row0 = [pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2),
#                 2 * q[1] * q[2] - 2 * q[0] * q[3],
#                 2 * q[1] * q[3] + 2 * q[0] * q[2]]
#         row1 = [2 * q[1] * q[2] + 2 * q[0] * q[3],
#                 pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2) - pow(q[3], 2),
#                 2 * q[2] * q[3] - 2 * q[0] * q[1]]
#         row2 = [2 * q[1] * q[3] - 2 * q[0] * q[2],
#                 2 * q[2] * q[3] + 2 * q[0] * q[1],
#                 pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2)]

        # quaternion from https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
        # oddly I had to multiply all of the matrix entries and then I got a seemingly correct
        # accel output where the vertical accel value is close to +9.81
        row0 = [-1 + 2 *(pow(q[0], 2) + pow(q[1], 2)), # same results as previous matrix
                2 * (q[1] * q[2] - q[0] * q[3]),
                2 * (q[1] * q[3] + 2 * q[0] * q[2])]
        row1 = [-2 * (q[1] * q[2] + q[0] * q[3]),
                -1 + 2 * (pow(q[0], 2) + pow(q[2], 2)),
                2 * (q[2] * q[3] - q[0] * q[1])]
        row2 = [2 * (q[1] * q[3] - q[0] * q[2]),
                2 * (q[2] * q[3] + q[0] * q[1]),
                -1 + 2 * (pow(q[0], 2) + pow(q[3], 2))]
        rm = [row0, row1, row2]
        return rm
    
    # initialize the quaternion at the beginning of data collection
    def initQuaternion(self, imu_data):
        a_mag = imu_data.accel_magnitude()
        z = [imu_data.accel.x / a_mag,
             imu_data.accel.y / a_mag,
             imu_data.accel.z / a_mag]

        x = [0] * 3
        if z[0] != 0:
            x_1 = numpy.sqrt(1 / (1 + pow(z[1], 2) / pow(z[2], 2)))
            x = [0, x_1, (-x_1 * z[1] / z[2])]
        elif z[1] != 0:
            x_0 = numpy.sqrt(1 / (1 + pow(z[0], 2) / pow(z[2], 2)))
            x = [x_0, 0, (-x_0 * z[0] / z[2])]
        else:
            x_0 = numpy.sqrt(1 / (1 + pow(z[0], 2) / pow(z[1], 2)))
            x = [x_0, (-x_0 * z[0] / z[1]), 0]

        y = numpy.cross(z, x)

        t = x[0]+y[1]+z[2]
        if t >= 0:
            r = numpy.sqrt(1+t)
            s = 0.5 / r
            self.quaternion[0] = 0.5 * r
            self.quaternion[1] = (z[1]-y[2]) * s
            self.quaternion[2] = (x[2]-z[0]) * s
            self.quaternion[3] = (y[0]-x[1]) * s
        elif x[0] >= y[1] and x[0] >= z[2]:
            r = numpy.sqrt(1+x[0]-y[1]-z[2])
            s = 0.5 / r
            self.quaternion[0] = (z[1]-y[2]) * s
            self.quaternion[1] = 0.5 * r
            self.quaternion[2] = (x[1]+y[0]) * s
            self.quaternion[3] = (z[0]+x[2]) * s
        elif y[1] > x[0] and y[1] >= z[2]:
            r =  numpy.sqrt(1 + y[1] - x[0] - z[2])
            s = 0.5 / r
            self.quaternion[0] = (x[2]-z[0]) * s
            self.quaternion[1] = (x[1]+y[0]) * s
            self.quaternion[2] = 0.5 * r
            self.quaternion[3] = (z[1]+y[2]) * s
        elif z[2] > x[0] and z[2] > y[1]:
            r =  numpy.sqrt(1 + z[2] - x[0] - y[1])
            s = 0.5 / r
            self.quaternion[0] = (y[0]-x[1]) * s
            self.quaternion[1] = (x[2]+z[0]) * s
            self.quaternion[2] = (y[2]+z[1]) * s
            self.quaternion[3] = 0.5 * r

    # convert the quarternion to euler angles
    def quaternionToEulerian(self):
        q = self.quaternion
        e = [float(0)] * 3
        # from vsf code
#         e[0] = math.atan2(2*(q[0]*q[1]+q[2]*q[3]),
#             q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])
#         e[1] = math.asin(2*(q[0]*q[2]-q[1]*q[3]))
#         e[2] = math.atan2(2*(q[0]*q[3]+q[1]*q[2]),
#             q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])
        # e definition from https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
        e[0] = math.atan2(2*(q[0]*q[1]+q[2]*q[3]),
            1.0-2.0*q[1]*q[1]+q[2]*q[2])
        t2 = 2*(q[0]*q[2]-q[3]*q[1])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        e[1] = math.asin(t2)
        e[2] = math.atan2(2*(q[0]*q[3]+q[1]*q[2]),
            1.0-2.0*q[2]*q[2]+q[3]*q[3])
        return e

    # convert euler angles to quaternion
    def eulerianToQuaternion(self, e):
        cos_eul = [float(0)] * 3
        sin_eul = [float(0)] * 3

        for i in range(3):
            cos_eul[i] = math.cos(e[i]/2.0)
            sin_eul[i] = math.sin(e[i]/2.0)
        # quaternion definition taken from vsf code and confirmed
        # from https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
        self.quaternion[0] = cos_eul[0]*cos_eul[1]*cos_eul[2]+ \
            sin_eul[0]*sin_eul[1]*sin_eul[2]
        self.quaternion[1] = sin_eul[0]*cos_eul[1]*cos_eul[2]- \
            cos_eul[0]*sin_eul[1]*sin_eul[2]
        self.quaternion[2] = cos_eul[0]*sin_eul[1]*cos_eul[2]+ \
            sin_eul[0]*cos_eul[1]*sin_eul[2]
        self.quaternion[3] = cos_eul[0]*cos_eul[1]*sin_eul[2]- \
            sin_eul[0]*sin_eul[1]*cos_eul[2]
  