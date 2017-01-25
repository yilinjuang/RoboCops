#! /usr/bin/env python

import datetime
import math
import time

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('test', anonymous=True)
rate = rospy.Rate(5)
cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
move_cmd = Twist()

while not rospy.is_shutdown():
    move_cmd = Twist()
    move_cmd.angular.z = math.radians(90)
    for t in range(0, 10):
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()
    time.sleep(3)
