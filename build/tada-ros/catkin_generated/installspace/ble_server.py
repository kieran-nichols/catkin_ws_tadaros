#!/usr/bin/env python3

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
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation
import threading

#EUROPA_ADDR=u"06:11"
EUROPA_ADDR=u"ef:1b"
#europa_file_path="/home/pi/Desktop/europa/data/"
class BleServer():
    def __init__(self):
        
        #rospy.Subscriber('kill_all_topic', Bool, handle_kill_command)
        print("initialized IN EUROPA")
        #MPU_Init()
        dev = EuropaBLE.EuropaBLE()   
        dev.set_device_addr(EUROPA_ADDR)
        dev.set_iface(0)
        #rospy.init_node('EuropaServer_node', anonymous=True)
        #dev.create_log_file(file_path=europa_file_path)
        #plot_thread=threading.Thread(target=plot_data,args=(dev,))
        #plot_thread.name="plot_thread handler"
        #plot_thread.setDaemon(True)
        #plot_thread.start()
        #europa_sensing = rospy.Publisher('europa_topic', String, queue_size=10)
        #rospy.init_node('europa_topic', anonymous=True)
        #rate = rospy.Rate(100)#100 hz
        while dev.search_europa()==0:
            dev.connect() #create new file for each connection
            dev.start_stream()
            dev.thread_process_data()
            #print(EuropaBLE.get_fz_data())
            #print(get_mz_data())
            #print("Reconnect")
            time.sleep(6)
            
            
        #if os.path.isdir(europa_file_path)==False:
        #os.makedirs(europa_file_path)
 
if __name__ == '__main__':
    try:
        BleServer()
        pass
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass