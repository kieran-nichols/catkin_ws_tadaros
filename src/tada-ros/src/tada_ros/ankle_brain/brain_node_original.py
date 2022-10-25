#!/usr/bin/env python

import rospy
from tada_ros.msg import MotorDataMsg,UserChoiceMsg, KillConfirmationMsg, IMUDataMsg
from std_msgs.msg import String
import os
import signal
from tada_ros.sensors import IMU_controller
from tada_ros.user_interface import user_choice_enum as enum
from tada_ros.global_info import constants

class BrainNode():
    motors_killed = False
    sensors_killed = False
    pub_movement = None
    pub_user = None
    current_IMU_data = None

    def __init__(self):
        # anonymous=True ensures that the name is unique by adding random numbers
        rospy.init_node('brain_node', anonymous=True)
        rospy.Subscriber('selection_topic', UserChoiceMsg, self.handle_user_input)
        rospy.Subscriber('sensing_topic', IMUDataMsg, self.handle_sensor_input)
        rospy.Subscriber('kill_confirmation_topic', KillConfirmationMsg, self.handle_kill_confirm)

        # queue_size argument limits the number of queued messages if any
        # subscriber is not receiving them fast enough.
        self.pub_movement = rospy.Publisher('movement_topic', String, queue_size=10)
        self.pub_user = rospy.Publisher('user_message_topic', String, queue_size=10)

        rospy.spin() # keeps python from exiting until this node is stopped

    def handle_user_input(self, data):
        user_choice = enum.UserChoiceEnum(data.choice)
        # for debugging
        print("")
        rospy.loginfo(rospy.get_caller_id() + " I heard choice: %s", \
            user_choice.name)
        if data.angle != constants.NO_ANGLE:
            rospy.loginfo(rospy.get_caller_id() + " I heard angle: %s", data.angle)
        print("")

        # just testing by sending the string for now
        if user_choice == enum.UserChoiceEnum.CURRENT_CONFIG or \
            user_choice == enum.UserChoiceEnum.TEST_GYRO:
            msg = None
            if self.current_IMU_data == None:
                msg = "sensors are not running"
            else:
                msg = "Current IMU Data:\n" + self.current_IMU_data.to_string()
            self.pub_user.publish(msg)
            # for debugging
            print("")
            rospy.loginfo(msg)
            print("")

        if user_choice != enum.UserChoiceEnum.KILL and \
            user_choice != enum.UserChoiceEnum.DEBUG and \
            user_choice != enum.UserChoiceEnum.CURRENT_CONFIG:
            self.pub_movement.publish(user_choice.name)

    def handle_sensor_input(self, msg_data):
        # translates IMUDataMsg ROS message to IMUData class and stores
        self.current_IMU_data = IMU_controller.ROS_message_to_IMUData(msg_data)

        # for debugging
        # print("IMU Data in Brain Node:")
        # IMU_data.print()
        # print("")

    def handle_kill_confirm(self, data):
        killed_string = ""
        if data.motors_killed:
            self.motors_killed = True
            killed_string = "motors "
        if data.sensors_killed:
            self.sensors_killed = True
            killed_string += "sensors"

        if self.motors_killed and self.sensors_killed:
            # interrupt the program like a Keyboard Interrupt
            os.kill(os.getpid(), signal.SIGTERM)

            # reset killed flags
            motors_killed = False
            sensors_killed = False

        # for debugging
        print("")
        rospy.loginfo(rospy.get_caller_id() + " I heard a confirmation that: %s was killed", killed_string)

if __name__ == '__main__':
    try:
        BrainNode()
    except rospy.ROSInterruptException: # ensures stopping if node is shut down
        pass
