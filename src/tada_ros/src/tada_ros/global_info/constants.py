#!/usr/bin/env python
import math
## IMU
# IMU reconstruction
STANCE_THRESHOLD = 50 # 50 [deg/s] angular velocity threshold for stance
GRAVITY = 9.81 #9.080297286843 # [m/s^2]
# code online: 9.8 * (1/16384) = .00059 VSF: 9.8 * (1/2048) = .00467516
# ACCEL_SCALE_FACTOR = GRAVITY * (1.0/2048.0) # [meters/(second^2)]
ACCEL_SCALE_FACTOR = GRAVITY * (1.0/16384) # [meters/(second^2)]
# code online: (3.14/180) *  (1/131) = .00013316 VSF: (math.pi/180) * (1.0/16.4) = .00106
# GYRO_SCALE_FACTOR = (math.pi/180) * (1.0/16.4) # [radians/second]
GYRO_SCALE_FACTOR = (math.pi/180) * (1.0/131) # [radians/second]
DT = 0.01

# Thresholds
GYRO_THRESHOLD_VELOCITY = 0.8 # 0.8
GYRO_THRESHOLD_ORIENTATION = 0.55 # 0.55
ACCEL_THRESHOLD = 1.0 # changed from .8
LOW_MOTION_COUNT_THRESHOLD = 10
NONLOW_MOTION_COUNT_THRESHOLD = 5
VELOCITY_THRESHOLD = 0.4 # 0.2
PREV_SWING_TIME_THRESHOLD = 0.3 # [seconds]
DIFF_TIME_THRESHOLD = 0.001 # [seconds]

LOW_MOTION_THRESHOLD_SECONDS = LOW_MOTION_COUNT_THRESHOLD * DT
NONLOW_MOTION_THRESHOLD_SECONDS = NONLOW_MOTION_COUNT_THRESHOLD * DT

## MOTOR

# EUROPA

## OTHER
import math
MIN_ANGLE_CHOICE = 0
MAX_ANGLE_CHOICE = 10
NO_ANGLE = -1
