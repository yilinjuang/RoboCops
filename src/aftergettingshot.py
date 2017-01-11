#! /usr/bin/env python

import numpy as np

import laser_geometry
import rospy
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2

class AfterGettingShot:
    def __init__(self):
        self.laser_data = np.zeros(681, dtype=np.float32)
        self.initflag=true
        self.poi = np.zeros(681, dtype=np.float32)

    def callback(self, data):
        laser_data = np.asarray(data.ranges, dtype=np.float32)
        laser_data[isnan(laser_data)]=0.0
        laser_data[isinf(laser_data)]=1000.0
        if initflag:
            self.laser_data=laser_data
            initflag=false
            return
        for value in np.absolute(data.ranges-laser_data):
            if value<=0.03:
                value=0.0

        #poi = np.flatnonzero(data.ranges - laser_data)
        #for value in poi:
       # print(data.ranges)
       # print(laser_data)
        # TODO: deal with inf and nan
       # if np.isclose(laser_data, self.laser_data, rtol=0.0, atol=0.1,
        #        equal_nan=True):
        #    print("Different!")
            # TODO: transform laser data to point cloud with laser_geometry.
            # Take the points by their indices.
            # Convert point cloud2 to XYZ and transform from /laser_link to /map.

    def scanForObjects(self):
        rospy.Subscriber("/laser_scan",LaserScan,self.callback)


    def transform(self, laser_data):
        lp=laser_geometry.LaserProjection()
        pc2=lp.projectLaser(data)
        for point in point_cloud2.read_points(pc2):
            print(str(point))


if __name__ == '__main__':
    rospy.init_node('laser_scan', anonymous=True)
    a = AfterGettingShot()
    a.scanForObjects()
    rospy.spin()
