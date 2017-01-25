#! /usr/bin/env python

import numpy as np
from math import pi

import laser_geometry
import rospy
import tf
from sensor_msgs.msg import LaserScan

class AfterGettingShot:
    def __init__(self, main=None):
        self.main = main

        self.prev_laser_data = None
        self.difference = None
        self.poi = None
        self.target = None
        self.direction = None
        self.prev_stamp = rospy.Time.now()

        # Register listener for laser scan.
        rospy.Subscriber("/laser_scan", LaserScan, self._get_laser_data)

    def _get_laser_data(self, data):
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
            # print("No moving object!")
            pass
        else:
            # print("POI: " + str(self.poi))
            d_odom = (self.poi * 0.00613592332229 - 2.08621382713) / pi
            if d_odom < 0.0:
                d_odom += 1.0
                direction = 1.0
            else:
                d_odom -= 1.0
                direction = -1.0
            if self.main:
                target_odom = self.main.mover.odom + d_odom
            else:
                target_odom = d_odom
            if target_odom > 1.0:
                target_odom -= 2
            elif target_odom < -1.0:
                target_odom += 2
            self.target = target_odom
            self.direction = direction


if __name__ == '__main__':
    rospy.init_node('aftergettingshot', anonymous=True)
    a = AfterGettingShot()
    rospy.spin()
