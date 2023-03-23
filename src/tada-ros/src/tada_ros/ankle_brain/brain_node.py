#!/usr/bin/env python
import rospy
import os
import time
import sys
import signal
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from tada_ros.msg import MotorDataMsg
from tada_ros.msg import MotorListenMsg
from tada_ros.msg import IMUDataMsg
from tada_ros.msg import EuropaMsg
#from tada_ros.europa import EuropaBLE
from tada_ros.sensors import IMU_controller
from tada_ros.sensors import sensor_node
import numpy as np
import math
import os
import threading

class BrainNode():
    # motor_command = [0, 0, 180,180, 1500, 1500] # motor mode, time to complete, motor1 position, motor2 position, motor1 torque, motor2 torque
    # Motor mode: 0 is move one direction, 1 is move one direction and then move back to initial
    # Time to complete: if 0, move as fast as possible, if more than 0, move within that time (more helpful for swing
   
    # Initialization and declaration of global-like (self) variables
    def __init__(self):
        #rospy.init_node('sensor_node', anonymous=True)
        #rospy.init_node('IMU_controller', anonymous=True)
        rospy.init_node('brain', anonymous=True)
        #Subscribbing to some topics
        print("init brain")
        rospy.Subscriber('europa_topic', EuropaMsg, self.handle_europa_input)
        rospy.Subscriber('sensing_topic', IMUDataMsg, self.handle_sensor_input)
        rospy.Subscriber('gui_topic', String, self.GUI_input)

        #imu_sensor = sensor_node.SensorNode()
        #self.sensor_info = Sensor()
        # Publisher: motor
        self.pub = rospy.Publisher('motor_command', MotorDataMsg, queue_size=10)
                
        self.motor_command = MotorDataMsg()
        # Subscribers: motors, IMU, Europa; subscribe command with it necessary variables that will be attached to self
        self.sub = rospy.Subscriber('motor_listen', MotorListenMsg, self.listener)
        self.curr_pos1 = 0; self.curr_pos2 = 0
        #rospy.spin() # keeps python from exiting until this node is stopped
        ## Add rospy.Subscriber with self, its necessary global-like variables, and appropiate listeners (aka callback functions)
        #handling the sensor data
        # Initialize the global-like variables
        self.homed1 = 0; self.homed2 = 0
        self.cnts_per_rev = 567
        self.raw_var = [False]
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.state = 0
        self.swing_time = 0
        self.t = 0
        self.mx = 0
        self.my = 0
        self.fz = 0
        
        def input_thread():
            while not rospy.is_shutdown():
                # sleep function so other commands can be typed first
                # ~ time.sleep(0.01)
                print("type 'help' for description of full instructions")
                print("Enter command(s): ")
                self.raw_var = list(input().split())
#                 return raw_var
    
        input_thread = threading.Thread(target=input_thread, args=())
        input_thread.start()
    
    ## Functions for IMU and Europa
    def handle_sensor_input(self, data):
        # translates IMUDataMsg ROS message to IMUData class and stores
        try:
            self.accel_x = data.accel_x
            self.accel_y = data.accel_y
            self.accel_z = data.accel_z
            self.gyro_x = data.gyro_x
            self.gyro_y = data.gyro_y
            self.gyro_z = data.gyro_z
            self.state = data.state
            self.swing_time = data.swing_time
            self.t = data.t
        except: 
            print("no data from IMU")

    def handle_europa_input(self, data):
        # translates IMUDataMsg ROS message to IMUData class and stores
        try:
            self.mx = data.mx
            self.my = data.my
            self.fz = data.fz
        except: 
            print("no data from Europa")
    
    def GUI_input(self, msg_data):
        # translates IMUDataMsg ROS message to IMUData class and stores
        #print("handler trg")
        string = str(msg_data.data)
#         print("Enter command(s): ")
#         print(string)
        self.raw_var = list(string.split())

        ## Add variables for IMU and Europa here
        
    # defining the listener functions for the subscibed nodes
    def listener(self,data):
        self.curr_pos1 = data.curr_pos1
        self.curr_pos2 = data.curr_pos2
    
    # Main function that will continuous iterate; it will have a mixture of publishing and subscribing
    def action(self):
        # Initialize the local variables
        var1 = 0; var2 = 0
        # initialize global-like variables
        self.prev_var1 = 0; self.prev_var2 = 0
        self.prev_var3 = 0; self.prev_var4 = 0
        var = []
        self.mode = 0
        self.steps = 0
        self.prev_steps = 0
        self.start_time = time.perf_counter()
        self.swing_test = [0, 0]
        self.stance_tracker = 0
        self.prev_stance_tracker = 0
        self.initial_itr = 0
        self.initial_itr1 = 0
        self.PF = 0
        self.EV = 0
        self.load_threshold = 200
        self.elapsed_time = 0
        self.prev_stance_theta, self.prev_stance_alpha = 0,0
        self.home_multiple1 = 0; self.home_multiple2 = 0
#         rate_motor = rospy.Rate(10) # every 0.1 sec
        rate = rospy.Rate(100) # every 0.01 sec 
        # specify home when the motor is turned on to be the motor positions at start
        self.homed1 = self.curr_pos1; self.homed2 = self.curr_pos2
        
        theta_array = [2.5, 5, 7.5, 10]
        alpha_array = [0, 180, 0, 180, 0, 180, 0, 180] # only sagittal
        # ~ alpha_array = [-90, 90, -90, 90, -90, 90, -90, 90] # only frontal
        # ~ alpha_array = [-135, -90, -45, 0, 45, 90, 135, 180] # mixture of frontal and sagiattal
        # ~ self.mode = 0
        self.tada_v1_data = [[0,180]] # empty list that will hold the TADA_angle cmds
        self.itr_v1 = 0
        
        # create tada_v1 experiment theta, alpha command angles
        for i in range(1): # five sets of movements
            self.tada_v1_data.append([0.0, 180.0])
            for x in theta_array:
                for y in alpha_array:
                    # ~ print(x,y)
                    self.tada_v1_data.append([x,y])
        self.tada_v1_data.append([0.0, 180.0])
        # end with 0,0 giving a total of 34 unique combinations
                
        def limit(num, minimum, maximum):
            return max(min(float(num), float(maximum)), float(minimum))
            
        def TADA_angle(self):
            theta_deg = self.theta_deg; alpha_deg = self.alpha_deg
            homed1 = self.homed1; homed2 = self.homed2
            home_multiple1 = self.home_multiple1; home_multiple2 = self.home_multiple2 
            
            theta = theta_deg*math.pi/180
            beta = 5*math.pi/180
            q3 = 2*np.real((np.arccos(np.sin(theta/2)/np.sin(beta)))) # arccos in python always returns real values
#             alpha = np.arctan2(PF*math.pi/180, EV*math.pi/180)
            alpha = alpha_deg*math.pi/180
            M1 = 180/math.pi*(alpha - np.arctan2(np.tan(q3/2),np.cos(beta))) 
            M2 = 180/math.pi*(-(alpha + np.arctan2(np.tan(q3/2), np.cos(beta))))
            # print motor angles in non-TADA ref frame
            # ~ print("Motor angles from homed", M1, M2)
            
            # Wrapping function that ensures that the angle is between 180 and -180;
            ## need to finish verify
            # ~ M1 = np.degrees(np.arctan2(np.sin(np.radians(M1)), np.cos(np.radians(M1))))
            # ~ M2 = np.degrees(np.arctan2(np.sin(np.radians(M2)), np.cos(np.radians(M2))))
            # ~ M1 = (M1 + 180)%360 - 180
            # ~ M2 = (M2 + 180)%360 - 180
            # ~ print("Wrapped motor angles from homed", M1, M2)
            
            q1 = M1*np.pi/180;
            q5 = M2*np.pi/180;
            q2 = np.pi/36; q4 = q2;
            R01 = np.array([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0], [0, 0, 1]])
            R12 = np.array([[np.cos(q2), 0 , np.sin(q2)], [0, 1, 0], [-np.sin(q2), 0, np.cos(q2)]])
            q3 = -q1 - q5;
            R23 = np.array([[np.cos(q3), -np.sin(q3), 0], [np.sin(q3), np.cos(q3), 0], [0, 0, 1]])
            R34 = np.array([[np.cos(q4), 0, np.sin(q4)], [0, 1, 0], [-np.sin(q4), 0, np.cos(q4)]])
            R45 = np.array([[np.cos(q5), -np.sin(q5), 0], [np.sin(q5), np.cos(q5), 0], [0, 0, 1]])
            
            R02 = np.matmul(R01,R12)
            R03 = np.matmul(R02,R23)
            R04 = np.matmul(R03,R34)
            R05 = np.matmul(R04,R45)
            
            self.PF = float(180/np.pi*R05[0,2])
            self.EV = float(180/np.pi*R05[1,2])
            
            ## To minimize wrapping issues and automatic redifinition of homed if the motor moves too far from original homed
            # Ensure that M1 and M2 are moving the minimum path (less than 180 deg)
            # ~ print("M1,M2:",M1,M2)
            # M1
            if M1>180:
                delta_M1 = M1 - 360 
            elif M1<-180:
                delta_M1 = M1 + 360 
            else: delta_M1 = M1
            # M2
            if M2>180:
                delta_M2 = M2 - 360 
            elif M2<-180:
                delta_M2 = M2 + 360 
            else: delta_M2 = M2

            if abs(M1 - self.M1_prev)<180 & abs(M1) + abs(self.M1_prev)>180:
                self.home_multiple1 += 1
                delta_M1 = M1 - 360 
            elif M1<-180:
                delta_M1 = M1 + 360 
            else: delta_M1 = M1
            # M2
            if M2>180:
                delta_M2 = M2 - 360 
            elif M2<-180:
                delta_M2 = M2 + 360 
            else: delta_M2 = M2
            
            # Convert to counts for motor movement
            M1_counts = delta_M1*self.cnts_per_rev/360
            M2_counts = delta_M2*self.cnts_per_rev/360
            
            # Keep track of homed multiple
            # ~ self.home_multiple1 = round(homed1/self.cnts_per_rev)
            # ~ self.home_multiple2 = round(homed2/self.cnts_per_rev)
            
            # Enusre that M1 and M2 are close to homed position if not then redefine a new homed multiple
            # ~ proposed_moevement_from_homed1 = M1_counts - 0*(self.home_multiple1*self.cnts_per_rev)
            # ~ proposed_moevement_from_homed2 = M2_counts - 0*(self.home_multiple2*self.cnts_per_rev)
            # ~ if not -self.cnts_per_rev/2 < proposed_moevement_from_homed1 < self.cnts_per_rev/2: 
                # ~ self.home_multiple1 = round((0*homed1 + M1_counts)/self.cnts_per_rev)
            # ~ if not -self.cnts_per_rev/2 < proposed_moevement_from_homed2 < self.cnts_per_rev/2: 
                # ~ self.home_multiple2 = round((0*homed2 + M2_counts)/self.cnts_per_rev)
                
            # Create command for motors from user specified homed with a continous adjustment to ensure motor is close to a multiple of the homed value
            global_M1 = M1_counts + 0*self.home_multiple1*self.cnts_per_rev + homed1
            global_M2 = M2_counts + 0*self.home_multiple2*self.cnts_per_rev + homed2

            # ~ print("Global motor angles", global_M1, global_M2,"")
            
            return [global_M1, global_M2, self.PF, self.EV]
        
        # Test for sagittal only ankle movement from neutral to dorsiflexed and back to neutral
        # Input array: [theta (deg)   , alpha (deg)   , time (ms)]
        # Sample array: [10           , 45            , 300      ]
        # All alphas will be 0 or 180 to keep sagittal only movements (0 for plantarflexion and 180 for dorsiflexion)
        def move_swing(self):
            var1,var2 = 0,0
            # ~ self.initial_itr1 = 0
            # swing state
            #if self.fz < self.load_threshold: # for Europa load, there is an inconsistent delay so we won't used Europa for real-time feedback
            if self.state == 1: # swing
                if self.initial_itr1 == 0:
                    print("swing")
                    self.start_time = time.perf_counter()
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    self.initial_itr = 0
                    self.initial_itr1 = 1
                else:
                    now = time.perf_counter()
                    elapsed_time = now - self.start_time
                    # move to 5 deg dorsiflexed for 2/3 of the time
                    if elapsed_time <= self.swing_test[2]/2: # consider changing the time to be dependent on the average of the previous swing times
                        self.theta_deg = 10.0; self.alpha_deg = 0.0
                    # move back to original position
                    elif self.swing_test[2]/2 < elapsed_time <= self.swing_test[2]:
                        self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    else: 
                        self.initial_itr = 0
                        self.initial_itr1 = 1
                        # ~ self.start_time = time.perf_counter()
                        self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                        # ~ self.start_time = time.perf_counter()
                
            # stance
            else:
                if self.initial_itr == 0:
                    # ~ print(elapsed_time)
                    print("stance")
                    # ~ self.start_time = time.perf_counter()
                    # ~ self.prev_steps = self.steps 
                    # ~ self.steps += 1
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    self.initial_itr = 1
                    self.initial_itr1 = 0       
                else: 
                    # ~ self.steps += 1
                    # ~ self.prev_stance_tracker = self.stance_tracker
                    # ~ self.start_time = time.perf_counter()
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
            # stance state and everything else
            # ~ else: 
                #if self.initial_itr != 0:
                #print("stance")
                #self.prev_stance_tracker = self.stance_tracker
                #self.stance_tracker += 1
                # ~ self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                # ~ self.start_time = time.perf_counter()
            
            motor = TADA_angle(self)
            # ~ print(motor)
            var1 = round(motor[0]) 
            var2 = round(motor[1])
            var3 = float(motor[2])
            var4 = float(motor[3])
            return var1, var2, var3, var4
        
        def tada_v1_expt(self):
            var1,var2 = 0,0
            cmd = []
            now = time.perf_counter()
            toe_lift_time = 0.3
            
            # ~ if (self.mode == 1 or self.mode == 2):
            if self.itr_v1 < 34*1 and self.mode != 0: # number of total itr
                cmd = self.tada_v1_data[self.itr_v1]
                # ~ print("here")
                
                now = time.perf_counter()
                self.elapsed_time = now - self.start_time 
                    
                # mode 3 
                # move the motor to 10 DF for 0.15 s and return back to original position for the
                if self.mode == 3: 
                    total_time = 2 # 2 seconds 
                    # 0 - 0.10          
                    if self.elapsed_time <= toe_lift_time/2: # consider changing the time to be dependent on the average of the previous swing times
                        self.theta_deg = 10.0; self.alpha_deg = 180.0
                    # move back to original position
                    # 0.15 - 0.3
                    elif toe_lift_time/2 < self.elapsed_time < toe_lift_time:
                        self.theta_deg = cmd[0]; self.alpha_deg = cmd[1]
                    # 0.3 - 1.0
                    elif toe_lift_time < self.elapsed_time < 1:
                        self.theta_deg = cmd[0]; self.alpha_deg = cmd[1]
                    # at 1.0 sec move to new position
                    # 1.0 - 2
                    else: 
                        cmd1 = self.tada_v1_data[self.itr_v1+1]
                        self.theta_deg = cmd1[0]; self.alpha_deg = cmd1[1]
                # mode 2
                else:
                    total_time = 1 # one second
                    self.theta_deg = cmd[0]
                    self.alpha_deg = cmd[1]                
                
                # if elapsed time has passed, move to the next position, and reset the start timer
                if self.elapsed_time >= total_time: # time to wait
                    self.itr_v1 += 1
                    self.start_time = time.perf_counter()
                    # ~ print("move to next")
	
            # stop movement, return TADA to neutral, and return to default mode
            else:
                print("end tada_v1 expt"+ str(self.mode-1)) # expt 1 is mode 2, expt 2 is mode 3
                self.mode = 0   
                self.itr_v1 = 0
                self.theta_deg = 0
                self.alpha_deg = 0
                self.start_time = time.perf_counter()
      
            motor = TADA_angle(self)
            # ~ print(motor)
            var1 = round(motor[0]) 
            var2 = round(motor[1])
            var3 = float(motor[2])
            var4 = float(motor[3])
            return var1, var2, var3, var4
                
        # main loop that controls the TADA
        while not rospy.is_shutdown(): #and rospy.on_shutdown(hook):
            # repurpose and shorten the self variables to be local variables
            motor_command = self.motor_command
            curr_pos1 = self.curr_pos1; curr_pos2 = self.curr_pos2
            homed1 = self.homed1; homed2 = self.homed2
            ## variables that are called here are updated to be used in this while loop          
            # read input from the terminal; expecting between 1 and 3 inputs
            # assigns the raw input data to var and ignores error if no input was given
            if self.raw_var: 
                var = self.raw_var
            else:
                var = [0, 0] 
            
            # if first command is h then kill motor node; launch file will restart it in 2 seconds
            ## need smarter way to restart the motor node
            if len(var)== 2:
                if var[0]=="kill":
                    if var[1]=="m":
                        os.system("rosnode kill /motor")
                        print("motor node will restart\n")
    #                     os.system("roslaunch ~/catkin_ws/src/motor_node.launch") # does not work as intended
                    elif var[1]=="a":
                        os.system("rosnode kill -a")
                        os.system("killall -9 rosmaster")
                        print("ROS has been killed")
                    else:
                        print("Please enter options 'm' or 'a'\n")
                        
                elif var[0]=="bt":
                    self.load_threshold = int(var[1])     
                 
                 # if only 2 commands are given then calculate motor angle as a function of the ankle angles which are the inputs
                else:
                    theta = limit(var[0], 0, 10)
                    alpha = limit(var[1], -180, 180)
                    self.theta_deg = float(theta) # keep value between 0 and 10
                    self.alpha_deg = float(alpha) # keep value between -180 and 180
                    
                    # Convert input of PF, EV, inclination angle to motor angles from homed
                    motor = TADA_angle(self)
                    var1 = int(motor[0]) 
                    var2 = int(motor[1])
                    print("Moving to", var1, var2,"\n")
                
            elif len(var)== 1:
                # print statement to describe instructions
                if var[0]=="help":                
                    print("\nTo command motor movement: 'm num num' (num is motor ticks where 567 for full rev)")
                    print("To command ankle angles: 'theta(0 to 10 deg) alpha(-180 to 180 deg)'")
                    print("Use 'h' to home, 'r' to return to home, 'c' for print current position")
                    print("Use 'i' to read IMU, 'e' to read Europa, 'kill m' or 'kill a' to kill motor or all systems\n")
#                     self.raw_var = [False]
                    
                # if first command is h then assign current position as homed values
                ## need to fix homing the current position; it seems that the correct value comes in the future iteration
                if var[0]=="h":
                    self.homed1 = curr_pos1
                    self.homed2 = curr_pos2
                    print(curr_pos1, curr_pos2)
                    print("created new homed positions:", self.homed1, self.homed2,"\n")     
                
                # if first command is r then return the motors back to home
                elif var[0]=="r":
                # create new homed value and specify the movement toward the new home which should be the same position as the old home
    #                 var1 = cnts_per_rev*round(homed1*cnts_per_rev)
    #                 var2 = cnts_per_rev*round(homed2*cnts_per_rev)
                    var1 = homed1
                    var2 = homed2
                    print("returned to home\n")  
                
                # if first command is c then print the current motor positions
                elif var[0]=="c":
                    print("Current motor positions:", curr_pos1, curr_pos2,"\n")
                    var1 = curr_pos1 # 180
                    var2 = curr_pos2 # 567
                
                ## if first command is i then read the IMU's state
                elif var[0]=="i":
                    print(self.accel_x, self.accel_y, self.accel_z, self.gyro_x, self.gyro_y, self.gyro_z, self.state, self.swing_time, self.t) 
                
                ## if first command is i then read the Europa's sagittal and frontal moments
                elif var[0]=="e":
                    print(self.mx, self.my, self.fz)
                    print("europa threshold = ", self.load_threshold)
                
                elif var[0] == "mode0" or var[0] == "x":
                    print("return to mode 0")
                    self.mode = 0
                    var1 = self.prev_var1 #curr_pos1 
                    var2 = self.prev_var2 #curr_pos2 
                    
                elif var[0]== "mode1":
                    # Test for sagittal only ankle movement from neutral to dorsiflexed and back to neutral
                    # Input array: [theta (deg)   , alpha (deg)   , time (s)]
                    # Sample array: [10           , 45            , 0.3     ]
                    # All alphas will be 0 or 180 to keep sagittal only movements (0 for plantarflexion and 180 for dorsiflexion)
                    self.mode = 1
                    self.steps += 1
                    self.start_time = time.perf_counter()
                    # ~ self.swing_test = [float(var[1]), float(var[2]), 0, 0, 300, 3, 1, 0, 0]
                    self.swing_test = [self.prev_stance_theta, self.prev_stance_alpha, 0.3]
                    print("Starting swing mode")
                    # ~ var1 = self.prev_var1 #curr_pos1 
                    # ~ var2 = self.prev_var2 #curr_pos2 
                    
                elif var[0]=="expt1":
                    print("Starting TADA_v1 experiment #1")
                    self.mode = 2
                    
                elif var[0]=="expt2":
                    print("Starting TADA_v1 experiment #2")
                    self.mode = 3
                
                # do nothing
                else:
                    pass #print("Did nothing for enter cmd (2)")
                    
            elif len(var)== 3:             
                # if first command is m, then send global motor commands
                if var[0] == "m":
                    var1 = int(var[1]) 
                    var2 = int(var[2])
                # manually set the motor location of homed
                elif var[0] == "h":
                    self.homed1 = int(var[1]) 
                    self.homed2 = int(var[2])
                    var1 = int(var[1]) 
                    var2 = int(var[2])
                else:
                    pass #print("Did nothing for enter cmd (3)")
                               
            # else statement to keep motor position the same; consider also to return to home     
            else:
                var1 = self.prev_var1 #curr_pos1 
                var2 = self.prev_var2 #curr_pos2 
                # ~ prev_var1, prev_var2 = var1, var2 
                print("Kept current position", var1, var2,"\n")
            
            # cmd to refresh terminal entry variable
            self.raw_var = [False]
            
            # condition to test swing
            if self.mode == 1:
                theta = limit(self.swing_test[0], 0, 10)
                alpha = limit(self.swing_test[1], -180, 180)
                self.theta_deg = float(theta) # keep value between 0 and 10
                self.alpha_deg = float(alpha) 
                var1,var2, self.PF, self.EV = move_swing(self)
            elif self.mode == 2 or self.mode == 3:
                var1,var2, self.PF, self.EV = tada_v1_expt(self)
            # ~ elif self.mode == 3:
                # ~ var1,var2, self.PF, self.EV = tada_v1_expt_unique(self)
            else: 
                self.mode = 0
                # var1, var2 are defined previous to these if statements, if there is terminal output or its keeps the prev_var's
                
            # set up object to publish to motor node
            motor_command.mode = 0
            motor_command.duration = 0
            motor_command.motor1_move = int(var1)
            motor_command.motor2_move = int(var2)
            motor_command.motor1_torque = 1500 # be careful with torque values, max torque is dependent on the motor loads
            motor_command.motor2_torque = 1500 # 1000 for no load, above 1500 for assembled TADA
            motor_command.PF = self.PF
            motor_command.EV = self.EV
            motor_command.t = time.time()
            self.prev_var1 = var1
            self.prev_var2 = var2
            self.prev_var3 = self.prev_stance_theta # be careful not to confuse var1 and var2, and var2 and var4
            self.prev_var4 = self.prev_stance_alpha

#             rospy.loginfo(motor_command) 
            self.pub.publish(motor_command)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        BrainNode().action()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
