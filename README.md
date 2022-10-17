# catkin_ws_tadaros

# ROS controller for Two axis Adaptable Ankle (TADA) prosthetic device

## Prerequisites
* Raspberry Pi 4B
* Linux
* Python 3.7
* ROS Noetic

## Hardware
* Raspberry Pi
* Power source: 12 V Batteries with voltage regulators to give power to various devices
* Motor driver: Elmo gold twitter
* Motion sensor: Inertial Measurement Unit (IMU) of the GY-521 module MPU-6050
* Motors: Faulhaber BLDC motors with encoders (not used) and hall sensors
* Load cell: Europa+, Three Axis (Frontal and Sagittal moments, and axial force)

## Installation
* .img location link # TODO: Kieran do you have this?
* Install ROS: # TODO: does the .img link to the Raspberry Pi already have this?
  * http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* "source /home/pi/[CATKIN WORKSPACE FOLDER]/devel/setup.bash" # put this into
  the bashrc file (don't forget to open a new terminal to load the bashrc file)
* "git clone this repo as your a catkin workspace
* "catkin_make" # run from the base folder of the catkin workspace

## Project description
This codebase controls the TADA ankle prosthetic. It takes user commands to
direct the motors. It also takes in sensor input, which will eventually be
used to direct the motors without user input.

## How to use
* "roscore" # run the ROS master node from a terminal
* To run the TADA system, use "roslaunch launch_file.launch" from the catkin_ws_tadaros/src" folder
* To run individual nodes, use "rosrun tada_ros [NAME_NODE].py" ex: "rosrun tada_ros sensor_node.py" # run
   each node in a new terminal
* If you want to plot the data being sent across topics on a graph, you can use
  the command "rqt_plot topic1/field1:field2:field3" and then run the nodes.
  For example "rqt_plot recon_topic/pos_x:vel_x:accel_x"
  
## Issues and Error Handling
If you have any issues, please refer to the issue folder first as someone in our group could have dealt with it already. Also, please post your solutions to your general issues in there.

* When trying to run the nodes, I got an error on "import rospy".
I fixed it by editing line 160 of
/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py
from (e_errno, msg, *_) = e.args to (e_errno, msg) = e.args
(note: you will probably need to use sudo to get edit access, something like:
"sudo nano /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py"

## Authors
Kieran Nichols, Sofya Akhetova, Becca Roembke, Peter Adamczyk

## License
University of Wisconsin BADGER lab? # TODO

## Acknowledgements
Thanks to the current team and the past work from Ryan Moreno, Mike Greene, Preston Lewis,

## References

Copyright (c) <2022>, <Kieran Nichols>
All rights reserved.

This source code is licensed under the MIT-style license found in the
LICENSE file in the root directory of this source tree.
