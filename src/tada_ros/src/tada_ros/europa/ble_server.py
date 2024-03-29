#!/usr/bin/env python

import rospy
import os
import sys
import signal
from rospy.numpy_msg import numpy_msg
from tada_ros.msg import MotorDataMsg
from tada_ros.msg import MotorListenMsg
from tada_ros.msg import IMUDataMsg
from tada_ros.msg import EuropaMsg
from std_msgs.msg import Bool
from time import sleep
from std_msgs.msg import *
import EuropaBLE as eu
from tada_ros.europa import EuropaBLE
import time
import os
import threading

#EUROPA_ADDR=u"06:11"
EUROPA_ADDR=u"06:53" #f5:ea" # 06:53 02:8D
#europa_file_path="/home/pi/Desktop/europa/data/"
class BleServer():
    def __init__(self):
        
        #rospy.Subscriber('kill_all_topic', Bool, handle_kill_command)
        #print("initialized IN EUROPA")
        #MPU_Init()
        dev = EuropaBLE.EuropaBLE()   
        dev.set_device_addr(EUROPA_ADDR)
        dev.set_iface(1)
        
        while dev.search_europa()==0 and not rospy.is_shutdown():
            
            dev.connect() #create new file for each connection
            dev.start_stream()
            dev.thread_process_data()
            
 
if __name__ == '__main__':
    try:
        BleServer()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
