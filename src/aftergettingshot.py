#! /usr/bin/env python

import numpy as np

import laser_geometry
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import movement

class AfterGettingShot:
    def __init__(self):
        self.mover = movement.Movement()
        self.rate = rospy.Rate(2)

        self.prev_laser_data = None
        self.difference = None
        self.poi = None
        self.prev_stamp = rospy.Time.now()

        self.scan_for_objects()

    def callback(self, data):
        """
        angle_min: -2.08621382713
        angle_max: 2.08621382713
        angle_increment: 0.00613592332229
        """
        # Control the rate of subscription.
        time_stamp = data.header.stamp
        if time_stamp - self.prev_stamp < rospy.Duration(0.1):
            return
        self.prev_stamp = time_stamp

        laser_data = np.asarray(data.ranges, dtype=np.float32)
        laser_data[np.isnan(laser_data)] = 0.0
        laser_data[np.isinf(laser_data)] = 0.0
        if self.prev_laser_data is None:
            self.prev_laser_data = laser_data
            return
        self.difference = np.absolute(laser_data - self.prev_laser_data)
        self.difference[np.less_equal(self.difference, 0.05)] = 0.0
        start = end = -1
        highest_counter = 20
        for pos, value in enumerate(self.difference):
            if value != 0.0:
                if start == -1:
                    start = pos
                end = pos
            else:
                if end - start > highest_counter:
                    highest_counter = end - start
                    self.poi = (start + end) // 2
                start = end = -1
        if highest_counter == 20:
            print("No moving object!")
        else:
            print("POI: " + str(self.poi))
            key = raw_input("AAA")
            if key == "R":
                # self.mover.rotate(1.0 * self.poi / 681)
                self.mover.rotate(5.7)
                self.rate.sleep()
                self.mover.rotate(0)
        self.mover.rotate(1)

    def cb(self, data):
        self.odom = data.pose.pose.orientation.z
        # print(odom)

    def scan_for_objects(self):
        rospy.Subscriber("/laser_scan", LaserScan, self.callback)
        rospy.Subscriber('/odom', Odometry, self.cb)


if __name__ == '__main__':
    rospy.init_node('laser_scan', anonymous=True)
    AfterGettingShot()
    rospy.spin()
