#! /usr/bin/env python

import rospy
from kobuki_msgs.msg import MotorPower

def get_motor_info(data):
    print("Got motor info!")
    print(data)

rospy.init_node('test', anonymous=True)
rate = rospy.Rate(5)
rospy.wait_for_message("/mobile_base/commands/motor_power", MotorPower)
rospy.Subscriber("/mobile_base/commands/motor_power", MotorPower, get_motor_info)

while not rospy.is_shutdown():
    rate.sleep()
