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
import pickle

import numpy as np
import math
import os
import threading
import psutil
import random

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
        self.toff = 0
        self.motor_fail = 0
        self.t = 0
        self.mx = 0
        self.my = 0
        self.fz = 0
        self.CPU = [0.0, 0.0, 0.0, 0.0]
        self.rate = rospy.Rate(100) # every 0.01 sec
        
        #for tada_v2_expt
        self.tada_v2_taken_angles = []
        self.tada_v2_paused = 1
        self.tada_v2_neutral = True
        self.tada_v2_expt_num = 0
        self.tada_v2_current_rotation = [5, -90]
        self.print_once = True
        self.starting_step = 0
        self.first_run = True
        self.random_step = random.randint(0,4)
        
        def input_thread():
            while not rospy.is_shutdown():
                # sleep function so other commands can be typed first
                # ~ time.sleep(0.01)
                print("type 'help' for description of full instructions")
                print("Enter command(s): ")
                self.raw_var = list(input().split())
                #self.rate.sleep
        input_thread = threading.Thread(target=input_thread, args=())
        input_thread.start()
        
        # Need separate thread to collect CPU info
        # Documentation said to sample at minimum 0.1 sec
        def cpu_stats_thread():
            while not rospy.is_shutdown():
                # read individual core usuage over 0.1 sec (minimum)
                self.CPU = psutil.cpu_percent(interval=0.1, percpu=True)
                # ~ time.sleep(0.01)
        cpu_stats_thread = threading.Thread(target=cpu_stats_thread, args=())
        cpu_stats_thread.start()
    
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
            self.steps = data.steps
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
        self.raw_var = list(string.split())

        ## Add variables for IMU and Europa here
        
    # defining the listener functions for the subscibed nodes
    def listener(self,data):
        self.curr_pos1 = data.curr_pos1
        self.curr_pos2 = data.curr_pos2
        self.toff = data.toff
        self.motor_fail = data.motor_fail
    
    # Main function that will continuous iterate; it will have a mixture of publishing and subscribing
    def action(self):
        # Initialize the local variables
        var1 = 0; var2 = 0
        # initialize global-like variables
        self.prev_var1 = 0; self.prev_var2 = 0
        self.prev_var3 = 0; self.prev_var4 = 0
        var = []
        self.mode = 0
        self.mode_prev = 0
        self.steps = 0
        self.prev_steps = 0
        self.start_time = time.perf_counter()
        self.swing_test = [0, 0]
        self.stance_tracker = 0
        self.prev_stance_tracker = 0
        self.initial_itr = 0
        self.initial_itr1 = 0
        self.restart_itr = 0
        self.PF = 0
        self.EV = 0
        self.curr_PF = 0
        self.curr_EV = 0
        self.load_threshold = 200
        self.elapsed_time = 0
        self.prev_stance_theta, self.prev_stance_alpha = 0,0
        self.home_multiple1 = 0; self.home_multiple2 = 0
        # specify home when the motor is turned on to be the motor positions at start
        self.homed1 = self.curr_pos1; self.homed2 = self.curr_pos2
        self.global_M1 = 0; self.global_M2 = 0;
        self.prev_M1 = 0; self.prev_M2 = 0
        self.homed1prev=0;self.homed2prev=0
        
        theta_array = [2.5, 5, 7.5, 10]
        theta_45_length = math.sqrt(50)
        theta_45_array_v1 = [theta_45_length/4, theta_45_length/2, 3*theta_45_length/4, theta_45_length]
        # ~ alpha_array = [0, 180 , 0, 180, 0, 180, 0, 180] # only sagittal
        theta_array_v2 = [5, 10]
        theta_45_array_v2 = [theta_45_length/2, theta_45_length]
        # ~ alpha_array = [-90, 90, -90, 90, -90, 90, -90, 90] # only frontal
        alpha_array = [-135, -90, -45, 0, 45, 90, 135, 180] # mixture of frontal and sagiattal
        # ~ alpha_array = [0, 180, 0, 180, 0, 180, 0, 180] 
        # ~ self.mode = 0
        self.tada_v1_data = [[0, 0]] # empty list that will hold the TADA_angle cmds
        self.tada_v2_data = [[0, 0]]
        self.tada_v2_data_mini = [[0, 0], [10, 0], [10, 180], [10, 90], [10, -90]] 
        self.itr_v1 = 0
        
        # create tada_v1 experiment theta, alpha command angles
        load_data = 0
        if load_data == 0:
            for i in range(1): # nine sets of movements
                # ~ self.tada_v1_data.append([0, 0])
                for (x,z) in zip(theta_array, theta_45_array_v1):
                    for y in alpha_array:
                        if abs(y)/45!=0 and abs(y)%90!=0: self.tada_v1_data.append([z,y])
                        else: self.tada_v1_data.append([x,y])
                random.shuffle(self.tada_v1_data)
                self.tada_v1_data.append([0, 0])
            
            # print(self.tada_v1_data)
            # shuffle the data
            # ~ random.shuffle(self.tada_v1_data)
            # ~ # check if consecutive elements exist and swap if necessary
            # ~ for i in range(len(self.tada_v1_data)-2):
                # ~ if self.tada_v1_data[i][0] == self.tada_v1_data[i+2][0] and self.tada_v1_data[i][1] == self.tada_v1_data[i+2][1]:
                    # ~ # if consecutive elements exist, swap them
                    # ~ self.tada_v1_data[i], self.tada_v1_data[i+2] = self.tada_v1_data[i+2], self.tada_v1_data[i]
            # ~ # start and end with 0,0 giving a total of 34 unique combinations and add back last item
            # ~ self.tada_v1_data.insert(0, [0, 0])
            # self.tada_v1_data.append(self.tada_v1_data[-1])
            # ~ self.tada_v1_data.append([0, 0])
            
            print("\nTADA_v1 data", self.tada_v1_data)
            
            # save to pickle for reuse        
            with open("/home/pi/catkin_ws/src/tada-ros/src/tada_ros/ankle_brain/shuffled_tada_v1_data.pickle", "wb") as handle:
                pickle.dump(self.tada_v1_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        else:
            with open("/home/pi/catkin_ws/src/tada-ros/src/tada_ros/ankle_brain/shuffled_tada_v1_data.pickle", "rb") as handle:
                self.tada_v1_data =pickle.load(handle)    
                # ~ print(self.tada_v1_data)          
        
        # create tada_v2 experiment theta, alpha command angles
        # ~ for i in range(1): # five sets of movements
        for (x,z) in zip(theta_array_v2, theta_45_array_v2):
            for y in alpha_array:
                if abs(y)%45==0 and abs(y)%90!=0: self.tada_v2_data.append([z,y])
                else: self.tada_v2_data.append([x,y])
                # ~ print(x,y)
        print("\nTADA_v2 data", self.tada_v2_data)
        # end with 0,0 giving a total of 34 unique combinations
                
        def limit(num, minimum, maximum):
            return max(min(float(num), float(maximum)), float(minimum))
            
        def TADA_angle_read(self):
            theta_deg = self.theta_deg; alpha_deg = self.alpha_deg
            homed1 = self.homed1; homed2 = self.homed2
            home_multiple1 = self.home_multiple1; home_multiple2 = self.home_multiple2 
            
            theta = theta_deg*math.pi/180
            beta = 5*math.pi/180
            q3 = 2*np.real((np.arccos(np.sin(theta/2)/np.sin(beta)))) # arccos in python always returns real values
            alpha = alpha_deg*math.pi/180
            M1 = 180/math.pi*(alpha - np.arctan2(np.tan(q3/2),np.cos(beta))) 
            M2 = 180/math.pi*(-(alpha + np.arctan2(np.tan(q3/2), np.cos(beta))))
            
            # need to use curr motor positions from motor
            M1 = (M1 + 180)%360 - 180
            M2 = (M2 + 180)%360 - 180
            
            q1 = M1*np.pi/180;
            q5 = M2*np.pi/180;
            q2 = np.pi/36; q4 = q2;
            R01 = np.array([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0], [0, 0, 1]])
            R12 = np.array([[np.cos(q2), 0 , np.sin(q2)], [0, 1, 0], [-np.sin(q2), 0, np.cos(q2)]])
            # ~ q3 = -q1 - q5;
            R23 = np.array([[np.cos(q3), -np.sin(q3), 0], [np.sin(q3), np.cos(q3), 0], [0, 0, 1]])
            R34 = np.array([[np.cos(q4), 0, np.sin(q4)], [0, 1, 0], [-np.sin(q4), 0, np.cos(q4)]])
            R45 = np.array([[np.cos(q5), -np.sin(q5), 0], [np.sin(q5), np.cos(q5), 0], [0, 0, 1]])
            
            R02 = np.matmul(R01,R12)
            R03 = np.matmul(R02,R23)
            R04 = np.matmul(R03,R34)
            R05 = np.matmul(R04,R45)
            
            self.curr_PF = float(180/np.pi*R05[0,2])
            self.curr_EV = float(180/np.pi*R05[1,2])
            return self.curr_PF, self.curr_EV
            
        def TADA_angle(self):
            theta_deg = self.theta_deg; alpha_deg = self.alpha_deg
            homed1 = self.homed1; homed2 = self.homed2
            home_multiple1 = self.home_multiple1; home_multiple2 = self.home_multiple2 
            theta = theta_deg*math.pi/180
            beta = 5*math.pi/180
            q3 = 2*np.real((np.arccos(np.sin(theta/2)/np.sin(beta)))) # arccos in python always returns real values

            alpha = alpha_deg*math.pi/180
            # counterlockwise is positive 
            M1 = 180/math.pi*(alpha - np.arctan2(np.tan(q3/2),np.cos(beta))) 
            M2 = 180/math.pi*(-(alpha + np.arctan2(np.tan(q3/2), np.cos(beta))))
            
            #this is the code to find the rotation 
            #M1_prev is current position and can be any number
            #M1_new is what we want
            M1_new=M1; M2_new=M2
            
            temp1 = M1_new-self.prev_M1
            rot1 = temp1%360
            if rot1>180:
                rot1=rot1-360
                
            temp2 = M2_new-self.prev_M2
            rot2 = temp2%360
            if rot2>180:
                rot2=rot2-360
            
            self.prev_M1=M1; self.prev_M2=M2           
            # Wrapping function that ensures that the angle is between 180 and -180;
            ## need to finish verify
            # ~ M1 = (M1 + 180)%360 - 180
            # ~ M2 = (M2 + 180)%360 - 180
            
            q1 = M1*np.pi/180;
            q5 = M2*np.pi/180;
            q2 = np.pi/36; q4 = q2;
            R01 = np.array([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0], [0, 0, 1]])
            R12 = np.array([[np.cos(q2), 0 , np.sin(q2)], [0, 1, 0], [-np.sin(q2), 0, np.cos(q2)]])
            # ~ q3 = -q1 - q5;
            R23 = np.array([[np.cos(q3), -np.sin(q3), 0], [np.sin(q3), np.cos(q3), 0], [0, 0, 1]])
            R34 = np.array([[np.cos(q4), 0, np.sin(q4)], [0, 1, 0], [-np.sin(q4), 0, np.cos(q4)]])
            R45 = np.array([[np.cos(q5), -np.sin(q5), 0], [np.sin(q5), np.cos(q5), 0], [0, 0, 1]])
            
            R02 = np.matmul(R01,R12)
            R03 = np.matmul(R02,R23)
            R04 = np.matmul(R03,R34)
            R05 = np.matmul(R04,R45)
            
            self.PF = float(180/np.pi*np.arctan2(R05[0,2],R05[2,2])) #float(180/np.pi*R05[0,2])
            self.EV = float(180/np.pi*np.arctan2(R05[1,2],R05[2,2]))
            
            # Convert rotation in deg to rotation we need to move it in counts
            rot1_counts = rot1*self.cnts_per_rev/360
            rot2_counts = rot2*self.cnts_per_rev/360
          
            # ~ print('global_m1, rot1: ', self.global_M1,rot1_counts)
            
            if homed1=='' and homed2=='':
                self.global_M1 = rot1_counts + self.global_M1#+ 0*self.global_M1 - self.homed1prev  
                self.global_M2 = rot2_counts +self.global_M2#+ 0*self.global_M2 - self.homed2prev 
            elif homed1==0 and homed2==0:
                self.global_M1 = rot1_counts #+ 0*self.global_M1 - self.homed1prev  
                self.global_M2 = rot2_counts#+ 0*self.global_M2 - self.homed2prev
                self.homed1 = ''; self.homed2 = ''; # reset homed values to empty string
            else:
                self.global_M1 = rot1_counts + homed1#+ 0*self.global_M1 - self.homed1prev  
                self.global_M2 = rot2_counts + homed2#+ 0*self.global_M2 - self.homed2prev
                self.homed1 = ''; self.homed2 = ''; # reset homed values to empty string
                
            # ~ self.homed1prev=homed1;self.homed2prev=homed2
            # ~ print('PF_cmd, EV_cmd: ', round(self.PF,3), round(self.EV,3))

            return [self.global_M1, self.global_M2, self.PF, self.EV]
        
        # Test for sagittal only ankle movement from neutral to dorsiflexed and back to neutral
        # Input array: [theta (deg)   , alpha (deg)   , time (ms)]
        # Sample array: [10           , 45            , 300      ]
        # All alphas will be 0 or 180 to keep sagittal only movements (0 for plantarflexion and 180 for dorsiflexion)
        def move_swing(self):
            var1,var2 = 0,0
            # swing state
            if self.state == 1: # swing
                if self.initial_itr1 == 0:
                    print("swing")
                    self.start_time = time.perf_counter()
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    self.initial_itr = 0
                    self.initial_itr1 = 1
                else:#stance
                    now = time.perf_counter()
                    elapsed_time = now - self.start_time
                    print("finished swing")
                    print(elapsed_time)
                    print(self.swing_time)
                    # move to 5 deg dorsiflexed for 1/2 of the time
                    if elapsed_time <= self.swing_test[2]/2: # consider changing the time to be dependent on the average of the previous swing times
                        self.theta_deg = 10.0; self.alpha_deg = 0.0
                    # move back to original position
                    elif self.swing_test[2]/2 < elapsed_time <= self.swing_test[2]:
                        self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    else: 
                        self.initial_itr = 0
                        self.initial_itr1 = 1
                        self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                
            # stance
            else:
                if self.initial_itr == 0:
                    print("stance")
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
                    self.initial_itr = 1
                    self.initial_itr1 = 0       
                else: 
                    self.theta_deg = self.swing_test[0]; self.alpha_deg = self.swing_test[1]
            
            motor = TADA_angle(self)

            var1 = (motor[0]) 
            var2 = (motor[1])
            var3 = (motor[2])
            var4 = (motor[3])

            return var1, var2, var3, var4
        
        def tada_v1_expt(self):
            var1,var2 = 0,0
            cmd = []
            now = time.perf_counter()
            # ~ toe_lift_time = 0.3
            # ~ print(len(self.tada_v1_data))

            if self.itr_v1 < len(self.tada_v1_data) and self.mode != 0: # number of total itr
                cmd = self.tada_v1_data[self.itr_v1]
                now = time.perf_counter()
                self.elapsed_time = now - self.start_time 
                # ~ print(self.itr_v1)
                    
                # mode 3 
                # move the motor to 10 DF for 0.15 s and return back to original position for the
                if self.mode == 3: 
                    total_time = 2 # 2 seconds 
                    # 0 - 0.15          
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
                    print('PF, EV: (', round(self.PF,3), round(self.EV,3), "), movement num", self.itr_v1,"of", len(self.tada_v1_data))
	
            # stop movement, return TADA to neutral, and return to default mode
            else:
                print("end tada_v1 expt"+ str(self.mode-1)) # expt 1 is mode 2, expt 2 is mode 3
                self.mode = 0   
                self.itr_v1 = 0
                self.theta_deg = 0
                self.alpha_deg = 0
                self.start_time = time.perf_counter()
      
            motor = TADA_angle(self)

            var1 = motor[0]
            var2 = motor[1]
            var3 = motor[2]
            var4 = motor[3]
            
            return var1, var2, var3, var4
        
        # Sofya please add to this tada_v1 expt. You can use/adpat the code move_swing 
        # and tada_v1_expt functions
        
        # input: self
        # output: var1, var2, var3, var4 which are motor1_cmd, motor2_cmd, PF, EV
        def tada_v2_expt(self, input_angles):
            step_count = 6
            if self.first_run:
                self.first_run= False
                print("Experiment trial number: ", self.tada_v2_expt_num)
                print("5 Steps to turn around/walk in neutral")
                random_index = random.randint(0, len(input_angles)-1)
                self.tada_v2_current_rotation = input_angles[random_index]
                
            if len(self.tada_v2_taken_angles)>=len(input_angles):
                self.mode = 0
                self.tada_v2_taken_angles = []
                self.tada_v2_paused = 1
                self.tada_v2_neutral = True
                self.tada_v2_expt_num = 0
                self.first_run = True
                print()
                print("Experiment finished")
                return 0, 0, 0, 0
                
            if self.tada_v2_paused==0:
                if self.print_once:
                    print("Experiment paused. Please let the participant take a break.\n")
                    print("type 'up' to un-pause\n")
                    self.print_once = False
                return 0, 0, 0, 0

            
            if self.tada_v2_neutral:
                if (self.steps-self.starting_step)<=5:
                    return 0, 0, 0, 0
                self.tada_v2_neutral = False
                #SET ANGLE FOR ANGLED
                self.theta_deg = self.tada_v2_current_rotation[0]
                self.alpha_deg = self.tada_v2_current_rotation[1]
                self.random_step = random.randint(0,4)
                print("Walk for ", step_count+self.random_step, " for ", self.tada_v2_current_rotation)
                self.starting_step = self.steps
            else:                
                if (self.steps-self.starting_step)<=(step_count+self.random_step):                    
                    motor = TADA_angle(self)
                    var1 = round(motor[0]) 
                    var2 = round(motor[1])
                    var3 = float(motor[2])
                    var4 = float(motor[3])
                    return var1, var2, var3, var4
                    
                #exeriment for that rotation finished so change
                self.tada_v2_neutral = True
                self.tada_v2_taken_angles.append(self.tada_v2_current_rotation)
                self.tada_v2_expt_num+=1

                #Getting the next random rotation
                random_index = random.randint(0, len(input_angles)-1)
                while (input_angles[random_index] in self.tada_v2_taken_angles):
                    #this means that we are done
                    if len(self.tada_v2_taken_angles)>=len(input_angles):
                        return 0, 0, 0, 0
                    random_index = random.randint(0, len(input_angles)-1)
                print("exit from random")
                self.tada_v2_current_rotation = input_angles[random_index]
                
                #pause after the TADA changes non-neutral angles 3 times and neutral angles 4 times
                # ~ if(self.tada_v2_expt_num%3==0)&(self.tada_v2_expt_num/3>0):
                if(self.tada_v2_expt_num%3==0):
                    self.print_once = True
                    self.tada_v2_paused = 0
                else:
                    print("Experiment trial number: ", self.tada_v2_expt_num)
                    print("5 Steps to turn around/walk in neutral")
                #SET ANGLE FOR NEUTRAL
                self.starting_step = self.steps
                self.theta_deg = 0
                self.alpha_deg = 0
                 
            motor = TADA_angle(self)
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
                        # ~ time.sleep(0.1)
                        # ~ print(self.prev_var1, self.prev_var1)
                        # ~ motor_restart_command = f"sudo lxterminal -e taskset -c 0,1,2 rosrun x_soem simple_test {self.prev_var1} {self.prev_var1} 0 0 1000 1000 __name:=motor"
                        # ~ os.system(motor_restart_command)                    
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
                    var1 = motor[0] 
                    var2 = motor[1]
                    print("Moving to", var1, var2,"\n")
                
            elif len(var)== 1:
                # print statement to describe instructions
                if var[0]== "up":
                    self.tada_v2_paused = 1
                    self.starting_step = self.steps
                    print("Experiment trial number: ", self.tada_v2_expt_num)
                    print("5 Steps to turn around/walk in neutral")
                    self.mode = self.mode_prev
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
                    self.swing_test = [self.prev_stance_theta, self.prev_stance_alpha, 0.3]
                    print("Starting swing mode")

                    
                elif var[0]=="expt1":
                    print("Starting TADA_v1 experiment #1")
                    self.mode = 2
                    
                elif var[0]=="expt2":
                    print("Starting TADA_v1 experiment #2")
                    self.mode = 3
                
                elif var[0]=="expt3":
                    print("Starting TADA_v2")
                    self.starting_step = self.steps
                    self.mode = 4
                    self.mode_prev = 4
                    
                elif var[0]=="expt3mini":
                    print("Starting TADA_v2")
                    self.starting_step = self.steps
                    self.mode = 5
                    self.mode_prev = 5
                    
                # do nothing
                else:
                    pass #print("Did nothing for enter cmd (2)")
                    
            elif len(var)== 3:             
                # if first command is m, then send global motor commands
                if var[0] == "m":
                    var1 = float(var[1])
                    var2 = float(var[2])
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
                # get current PF and EV
                self.curr_PF, self.curr_EV = TADA_angle_read(self)

            elif self.mode == 4:
                var1, var2, self.PF, self.EV = tada_v2_expt(self, self.tada_v2_data)
                self.mode_prev = 4
                
            elif self.mode == 5:
                var1, var2, self.PF, self.EV = tada_v2_expt(self, self.tada_v2_data_mini)
                self.mode_prev = 5
            else: 
                self.mode = 0
                
                
            # time
            current_time = rospy.Time.now()
            current_time_value = current_time.to_sec()
            current_time_value = current_time_value%100000
            
            # need to create int of motor commands
            var1 = int(round(var1))
            var2 = int(round(var2))
            
            # if motor has continous errors, the brain node will restart it
            # also if toff is too large; it should mostly be less than 20000 magnitude (20 us)
            if self.motor_fail == 1: #or abs(self.toff) > 45000:
                os.system("rosnode kill /motor")
                # ~ time.sleep(0.1)
                # ~ motor_restart_command = f"sudo lxterminal -e taskset -c 0,1,2 rosrun x_soem simple_test {self.prev_var1} {self.prev_var1} 0 0 1000 1000 __name:=motor"
                # ~ os.system(motor_restart_command)                
                self.restart_itr += 1
                print("motor node will restart, it has restarted times", self.restart_itr, "times\n")
            
            # set up object to publish to motor node
            motor_command.mode = 0
            motor_command.duration = 0
            motor_command.motor1_move = var1 # round the value then make it into an int
            motor_command.motor2_move = var2
            motor_command.motor1_torque = 1500 # be careful with torque values, max torque is dependent on the motor loads
            motor_command.motor2_torque = 1500 # 1000 for no load, above 1500 for assembled TADA
            motor_command.PF_cmd = self.PF
            motor_command.EV_cmd = self.EV
            motor_command.PF_curr = self.curr_PF
            motor_command.EV_curr = self.curr_EV
            [motor_command.CPU0, motor_command.CPU1, motor_command.CPU2, motor_command.CPU3] = self.CPU
            motor_command.t = current_time_value
            motor_command.valid = self.tada_v2_paused
                        
            self.prev_var1 = var1
            self.prev_var2 = var2
            self.prev_var3 = self.prev_stance_theta # be careful not to confuse var1 and var2, and var2 and var4
            self.prev_var4 = self.prev_stance_alpha

            self.pub.publish(motor_command)
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        BrainNode().action()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass

