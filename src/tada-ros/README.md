# ROS controller for Two axis Adaptable Ankle (TADA) prosthetic device

## Goals

## Prerequisites
* Raspberry Pi
* Linux
* Python 3.7
* ROS Noetic

## Hardware
* Raspberry Pi
* IMU: GY-521 module MPU-6050
* Motors: TBD

## Installation
* .img location link # TODO: Kieran do you have this?
* Install ROS: # TODO: does the .img link to the Raspberry Pi already have this?
  * http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* "source /home/pi/[CATKIN WORKSPACE FOLDER]/devel/setup.bash" # put this into
  the bashrc file (don't forget to open a new terminal to load the bashrc file)
* "git clone https://[USERNAME]@bitbucket.org/uwbadgerlab/tada-ros.git" # clone this
  inside a catkin workspace
* "catkin_make" # run from the base folder of the catkin workspace

## Project description
This codebase controls the TADA ankle prosthetic. It takes user commands to
direct the motors. It also takes in sensor input, which will eventually be
used to direct the motors without user input.

## How to use
* "roscore" # run the ROS master node from a terminal
* "rosrun tada_ros [NAME_NODE].py" ex: "rosrun tada_ros sensor_node.py" # run
   each node in a new terminal
  * run sensor_node from tada_ros/src/tada_ros so that the reconstruction files
    are organized properly
* If you want to plot the data being sent across topics on a graph, you can use
  the command "rqt_plot topic1/field1:field2:field3" and then run the nodes.
  For example "rqt_plot recon_topic/pos_x:vel_x:accel_x"

## Environments

## Code structure
* top level folders
  * global_info: has the constants used throughout the codebase for ease of
    updating
  * user_interface: contains user_node which takes user input to manipulate the
    ankle
  * ankle_brain: contains brain_node which processes user input and tells the
    motors how to behave
  * sensors: contains sensor_node which reads and processes IMU data;
    IMU_controller contains details of reading IMU data and reconstructing the
    data into position and velocity
  * motors: contains motor_node which directs the motors;
    currently just reads the commands and prints them out since we don't have
    hardware yet

* topics
  * selection_topic: user_node tells brain_node what selection the user made
  * sensing_topic: sensor_node tells brain_node the current IMU data
  * recon_topic: sensor_node sends the IMU reconstruction data (current
    position, velocity, and acceleration). Currently no one listening,
    it is being used for testing purposes
  * kill_all_topic: user_node tells sensor_node and motor_node to shut down
  * kill_confirmation_topic: sensor_node and motor_node tells brain_node that
    sensors or motors have been successfully shut down so that brain_node can
    shut down as well
  * user_message_topic: brain_node sends a message to user_node so that user_node
    can print to the user

## Issues and Error Handling
If you have any issues, please refer to the issue folder first as someone in our group could have dealt with it already. Also, please post your solutions to your general issues in there.

* When trying to run the nodes, I got an error on "import rospy".
I fixed it by editing line 160 of
/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py
from (e_errno, msg, *_) = e.args to (e_errno, msg) = e.args
(note: you will probably need to use sudo to get edit access, something like:
"sudo nano /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py"

## Authors
Peter Adamczyk, Ryan Moreno, Kieran Nichols, Becca Roembke

## License
University of Wisconsin BADGER lab? # TODO

## Acknowledgements
Thanks to the current team and the past work from Mike Greene, Preston Lewis,
... # TODO

## References

## Next Steps
* Motor specific code once we get the motor hardware
* See if we need IMU calibration

## Other
* Open Tasks/Active Questions
  * label where these equations are from: quaternionToEulerian and eulerianToQuaternion
  * how should the linear velocity correction work? In the VSF controller there is
    a complicated error correction for the position. I (Ryan) changed it to just
    set the position to the previous position if we're in stance.
  * do we need to shut down the IMU?
