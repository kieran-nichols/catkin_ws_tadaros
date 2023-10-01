# IMU
## This folder comprises files designed for interfacing with the IMU and report it's values to ROS. We are employing the MPU6050 as our inertial measurement unit.

There are two main files: IMU_controller.py and sensor_node.py

Parts of IMU_controller.py were adapted from the official source on pypi.org (https://pypi.org/project/mpu6050-raspberrypi/). To utilize this script, it's essential to have mpu6050-raspberrypi version 1.2 installed. The primary purpose of this script is to establish functions for extracting IMU values, as well as calculating swing time and step count based on those values. Any reporting in ROS is handled within sensor_node.py.

The creation of sensor_node.py aimed to transmit IMU values, along with swing time, stance/swing state, and step count, to our ROS system. It achieves this by utilizing the functions from IMU_controller and subsequently publishing the data to ROS.
