#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg
from tada_ros.msg import MotorDataMsg
from tada_ros.msg import MotorListenMsg
import numpy

class BrainNode():
    # motor_command = [0, 0, 180,180, 1500, 1500] # motor mode, time to complete, motor1 position, motor2 position, motor1 torque, motor2 torque
    # Motor mode: 0 is move one direction, 1 is move one direction and then move back to initial
    # Time to complete: if 0, move as fast as possible, if more than 0, move within that time (more helpful for swing
   
    # Initialization and declaration of global-like (self) variables
    def __init__(self):
        rospy.init_node('brain', anonymous=True)
        # Publisher: motor
        self.pub = rospy.Publisher('motor_command', MotorDataMsg, queue_size=10) 
        self.motor_command = MotorDataMsg()
        # Subscribers: motors, IMU, Europa; subscribe command with it necessary variables that will be attached to self
        self.sub = rospy.Subscriber('motor_listen', MotorListenMsg, self.listener)
        self.curr_pos1 = 0; self.curr_pos2 = 0
        ## Add rospy.Subscriber with self, its necessary global-like variables, and appropiate listeners (aka callback functions)
        
    # defining the listener functions for the subscibed nodes
    def listener(self,data):
        self.curr_pos1 = data.curr_pos1
        self.curr_pos2 = data.curr_pos2
    ## Add functions for IMU and Europa
    
    # Main function that will continuous iterate; it will have a mixture of publishing and subscribing
    def action(self):
        # Initialize the local variables
        ## Add variables for IMU and Europa here
        var1 = 0; var2 = 0
        prev_var1 = 0; prev_var2 = 0
        homed1 = 0; homed2 = 0
        cnts_per_rev = 567
        var = []
        rate = rospy.Rate(10) # every 0.1 sec
        
        # main loop that controls the TADA
        while not rospy.is_shutdown(): #and rospy.on_shutdown(hook):
            # repurpose and shorten the self variables to be local variables
            motor_command = self.motor_command
            curr_pos1 = self.curr_pos1; curr_pos2 = self.curr_pos2
            ## add IMU and Europa variables here that need to be read
            ## variables that are called here are updated to be used in this while loop
                        
            # read input from the terminal; expecting between 1 and 3 inputs
            raw_var = list(input("Enter motor 1 and 2 movement: ").split())
            
            # assigns the raw input data to var and ignores error if no input was given
            if raw_var: 
                var = raw_var
            else:
                var = [False] 
            
            # if first command is h then assign current position as homed values
            if var[0]=="h":
                homed1 = curr_pos1
                homed2 = curr_pos2
#                 print(curr_pos1, curr_pos2)
                print("created new homed positions:", homed1, homed2,"\n")     
            
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
            
            ## if first command is i then read the IMU's state
            elif var[0]=="i":
                print() 
            
            ## if first command is i then read the Eropa's sagittal and frontal moments
            elif var[0]=="e":
                print() 
                  
            # else; expecting
            elif len(var)==2:
                var1 = int(var[0]) 
                var2 = int(var[1])
                print("Moving to", var1, var2,"\n")
                
            # else statement to keep motor position the same; consider also to return to home     
            else:
                var1 = curr_pos1 # 180
                var2 = curr_pos2 # 567
                print("Kept current position", var1, var2,"\n")
            
            # set up object to publish to motor node
            motor_command.mode = 0
            motor_command.duration = 0
            motor_command.motor1_move = var1
            motor_command.motor2_move = var2
            motor_command.motor1_torque = 1000 # be careful with torque values, max torque is dependent on the motor loads
            motor_command.motor2_torque = 1000 # 1000 for no load, above 1500 for assembled TADA
            prev_var1 = var1
            prev_var2 = var2
#             rospy.loginfo(motor_command) 
            self.pub.publish(motor_command)
            rate.sleep()    
    
if __name__ == '__main__':
    try:
        BrainNode().action()
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
