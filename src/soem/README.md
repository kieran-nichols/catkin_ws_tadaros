# SOEM for TADA ROS

## Motor node (ROS)
* Position control of two brushless DC motors 
* Faulhaber info (need to add)
* Multi-threaded C/C++ program
* To compile program, run catkin_make in main directory (catkin_ws_tadaros)
* Input: Brain messages (first 6 indices)
* Output: Motor information 
* Functions: to set up and execute motor position control over CANopen over EtherCAT

## Resources: this code was edited using files from 
1) SOEM 
2) ROS wrapper for SOEM
3) Some help from mgruler, others

CPU usage
1) Linux scheduling, Kernel operations
2) ROS, TADA programs, error tracking (100Hz), movement thread (100Hz)
3) ROS, TADA programs, error tracking (100Hz), movement thread (100Hz)
4) synch thread (1000 Hz)

## Motor communication Description
### Main thread
	Variables initialization
	Macro functions for SDO/PDO reading and writing
	Initialize threads for general communication, error tracking, ROS messaging, Synchronization

### General communication thread
	Initial ethernet connection and initialize Master
	Move from init mode t
	Pre-Operational mode
	Set SDO and PDO mapping for slaves (set CSP mode and specify the correct addresses for the read and write of position, velocity, and torque)
	Move to Safe Operational mode
	Move to Operational mode and ensure the state machine is ready
	Operational mode: Position control
		While loop
		i++
		Send movement data from previous iteration
		If i == 1: set initial and final positions
		If else move mode is true: 
			read current position, 
			set target position (send info to synch thread)
			and check if movement is finished
		Else: stay at initial position
		Sleep for 500 microseconds (0.5 ms)
	Move to Pre-Operational mode
	Move to Init mode
	Close ethernet connection
	Exit program

### Error tracking
	More to add

### ROS messaging
	More to add

### Synch thread
	get current time to nearest millisecond
	convert to nanoseconds
	initialize variables, toff, cycletime, dorun
	toff: time to wait to start next cycle
	cycletime: sampling period
	dorun: conditional statement that states if the system is in operational mode
	send initial ethercat data
	
	enter infinite while loop
	calculate time to start next cycle (addtime) based on cycletime and toff
	wait function to start next cycle
	if system is in operational mode
	  start mutex lock
	  send ethercat data
	  if first client has distribute clock value?
	    find curr_dc time with some accommodation from either 32 or 64 bit values (SOEM takes in 64 bit clock values but some drivers give 32 bit)
	    find delta which is time difference from 50 us sampling period continuously offset 50 us
	      Trying to ensure that linux clock is 50 us offset from ethercat clock
	    if delta is more than sampling period/2 then subtract sampling period from delta (it is closer to start of next period)
	    
	    if delta is positive, add one to the integral gain
	    if delta is negative, minus one to the integral gain
	    calculate toff to get linux time and DC synced
	      toff = -0.1(delta+integral)
	      toff = -0.1*(25+5)=-3  
	      
	Send process data to ethercat system
	Unlock the mutex
	
	
