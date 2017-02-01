#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

def count(dataset):
    nan = 0
    close = 0
    far = 0
    for data in dataset:
        #  print(data, end="\t")
        if math.isnan(data):
            #  print "NaN"
            nan += 1
            continue
        if data < 1.0:
            #  print "Close"
            close += 1
        else:
            far += 1
    return nan, close, far

def parse(dataset):
    #  left1 = dataset[:127]
    #  left2 = dataset[128:255]
    #  middle = dataset[255:383]
    #  right2 = dataset[384:511]
    #  right1 = dataset[512:]
    left = dataset[:220]
    middle = dataset[221:440]
    right = dataset[441:]

    print(np.nansum(middle) / 128)
    nan, close, far = count(middle)
    print("Close = " + str(close) + " Nan = " + str(nan) + " Far = " + str(far))
    dist = np.nansum(middle) / 128
    if dist < 2.0:
        return 0
    else:
        return 1
    #  nan, close, far = count(left1)
    #  if close > far:     # Close to the left
        #  return 1
    #  return 2

def callback(data):
    #[sensor_msgs/LaserScan]:
    # std_msgs/Header header
    #   uint32 seq
    #   time stamp
    #   string frame_id
    # float32 angle_min
    # float32 angle_max
    # float32 angle_increment
    # float32 time_increment
    # float32 scan_time
    # float32 range_min
    # float32 range_max
    # float32[] ranges (size = 640)
    # float32[] intensities
    #  print(data.angle_min)
    #  print(data.angle_max)
    #  print(data.angle_increment)
    rospy.loginfo(len(data.ranges))
    cmd = Twist()
    condition = parse(data.ranges)
    if condition == 0:      # Close to the right
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0 * math.pi / 90 * 10
        print("Close to the right")
    #  elif condition == 1:    # Close to the left
        #  cmd.linear.x = 0.05
        #  cmd.angular.z = -1.0 * math.pi / 90 * 20
        #  #  print("Close to the left")
        #  print("Far to the right")
    else:                   # OK
        cmd.linear.x = 0.1
        print("OK")
    pub.publish(cmd)

rospy.init_node('mover', anonymous=True)
rospy.Subscriber('kinect_scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
rate = rospy.Rate(1) # 1hz
rospy.spin()
