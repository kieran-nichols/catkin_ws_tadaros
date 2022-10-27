#!/usr/bin/env python3
import rospy
import os
import sys
import signal
from tada_ros.msg import KillConfirmationMsg, IMUDataMsg, ReconDataMsg
from std_msgs.msg import Bool
from time import sleep
from std_msgs.msg import String
from tada_ros.sensors import IMU_controller

class SensorNode():
    pub_killed_confirm = None
    # top position, top velocity, bottom position, bottom velocity
    #top_from_home , bottom_from_home , top_from_zero , bottom_from_zero
    #bottom_home, top_home ,  stance , bottom_zero, top_zero
    # stance = 50; # [deg/s] angular velocity threshold for stance

    def __init__(self):
        # killing the program is its own topic so that there's minimal delay
        rospy.Subscriber('kill_all_topic', Bool, self.handle_kill_command)

        # queue_size argument limits the number of queued messages if any
        # subscriber is not receiving them fast enough.
        self.pub_killed_confirm = rospy.Publisher('kill_confirmation_topic', Bool, queue_size=10)

        pub_sensing = rospy.Publisher('sensing_topic', String, queue_size=10)
        pub_recon = rospy.Publisher('recon_topic', ReconDataMsg, queue_size=10)
        # anonymous=True ensures that the name is unique by adding random numbers
        rospy.init_node('sensor_node', anonymous=True)
        rate = rospy.Rate(100) # 10hz

        # initialize the IMU controller
        imu = IMU_controller.IMUController()
        
        # start the imu with z axis vertical to get initial bias
        bias = imu.get_bias()

        first_time = True
        while not rospy.is_shutdown():
            # get sensor input; uncomment bias if you want live bias data
            #bias = imu.get_bias()
            imu_data = imu.get_data(bias)

            # do the IMU reconstruction
            recon_data = imu.imu_reconstruction(imu_data,bias)

            # send sensor input to topic
            msg_imu = imu_data.to_string()
            pub_sensing.publish(msg_imu)

            # send recon output to topic
            msg_recon = recon_data.to_ROS_message()
            pub_recon.publish(msg_recon)

            rate.sleep() # sleeps to maintain selected rate

    def handle_kill_command(self, data):
        # for debugging
        print("")
        rospy.loginfo(rospy.get_caller_id() + " I heard kill boolean: %s", data.data)
        print("")
        if data.data == True:
            self.kill_sensors()

        # send confirmation message
        msg = KillConfirmationMsg(motors_killed=True, sensors_killed=False)
        self.pub_killed_confirm.publish(msg)

        # interrupt the program like a Keyboard Interrupt
        os.kill(os.getpid(), signal.SIGTERM)

    def kill_sensors(self):
        # TODO: sensor-specific code
        print("Killing Sensor Node")

if __name__ == '__main__':
    try:
        SensorNode()
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
