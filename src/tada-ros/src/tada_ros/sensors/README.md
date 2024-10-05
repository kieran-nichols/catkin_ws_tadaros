# IMU

## ROS controller for Two axis Adaptable Ankle (TADA) prosthetic device

### Prerequisites
* Raspberry Pi 4B
* Linux
* Python 3.7
* ROS Noetic

### Hardware
* Raspberry Pi
* Power source: 12 V Batteries with voltage regulators to give power to various devices
* Motion sensor: Inertial Measurement Unit (IMU) of the GY-521 module MPU-6050This folder comprises files designed for interfacing with the IMU and report it's values to ROS. We are employing the MPU6050 as our inertial measurement unit.

### Software
There are two main files: IMU_controller.py and sensor_node.py

The primary purpose of this script is to establish functions for extracting IMU values, as well as calculating swing time and step count based on those values. Any reporting in ROS is handled within sensor_node.py.

The creation of sensor_node.py aimed to transmit IMU values, along with swing time, stance/swing state, and step count, to our ROS system. It achieves this by utilizing the functions from IMU_controller and subsequently publishing the data to ROS.

### Main functions
#### sensor_node.py
	Class SenorNode():
		Start and run ROS node for IMU
		Start and run IMUController Python object (see below)
		Initialize MPU6050 bus
		Initialize function that waits to fill this node

#### IMU_controller.py
	class IMUController():
		Start and run IMU's I2C bus
		Read and process the IMU's raw data for accel and gyro
		Calculate if the foot is in swing or stance, and determine swing time

