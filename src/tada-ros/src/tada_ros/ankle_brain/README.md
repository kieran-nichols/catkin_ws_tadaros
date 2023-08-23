catkin_ws/src/tada-ros/ankle_brain BrainNode.py
ROS Brain node basics
•	ROS initialization
•	User interface
•	Conversion of control TADA angles to motor angles
•	Automated experiment execution


Internal referenced commands
Def BrainNode():
	Def __init__ (self)
		def input_thread():
def cpu_stats_thread():## Functions for IMU and Europa
def handle_sensor_input(self, data):
def handle_europa_input(self, data):
def GUI_input(self, msg_data): # translates IMUDataMsg ROS message to IMUData class and stores
def listener(self,data): # defining the listener functions for the subscribed nodes
def action(self): # Main function that will continuous iterate; it will have a mixture of publishing and subscribing
               		def limit(num, minimum, maximum):
           				return max(min(float(num), float(maximum)), float(minimum))
def TADA_angle_read(self):
	return self.curr_PF, self.curr_EV
def TADA_angle(self):
	return [self.global_M1, self.global_M2, self.PF, self.EV]
def move_swing(self): #toe lift stuff!! # Test for sagittal only ankle movement from neutral to dorsiflexed and back to neutral    # Input array: [theta (deg)   , alpha (deg)   , time (ms)]    # Sample array: [10           , 45            , 300      ]    # All alphas will be 0 or 180 to keep sagittal only movements (0 for plantarflexion and 180 for dorsiflexion)
	return var1, var2, var3, var4
def tada_v1_expt(self):
return var1, var2, var3, var4  # output: var1, var2, var3, var4 are motor1_cmd, motor2_cmd, PF, EV
       			def tada_v2_expt(self, input_angles):
				return var1, var2, var3, var4



Commands:
“help”: brings up the help menu
“kill m”: kills just the motors
“kill a”: kills everything
“m ### ###”: put in the motor counts for both motors, something between 0 to 567 or something 
“h ### ###”: sets a certain motor count to the home position
“h”=set current position to home?
“r”= returns it to home
“## ###” (something between 0-10, something between -180 to 180
“expt1”:TADA_v1 experiment #1……… self.mode = 2
“expt2”: TADA_v1 experiment #2………..self.mode = 3
“expt3”: TADA_v2……… self.mode = 4  
for x,z in (theta_array_v2, theta_45_array_v2):
            for y in alpha_array:
                if int(y)%45!=0: self.tada_v2_data.append([x,y])
                else: self.tada_v2_data.append([z,y])
        # end with 0,0 giving a total of 34 unique combinations
“expt3mini": TADA_v2………..self.mode = 5     [0, 0], [10, 0], [10, 180], [10, 90], [10, -90]

“bt”: Europa threshold for historic swing
“up”: unpause expt3 and expt3mini
“c”: spits out current motor position
“i”: spits out current IMU position
“e”: spits out current Europa position
“x” : exit current mode, not great…
“mode1” : unknown, Kieran HELP
