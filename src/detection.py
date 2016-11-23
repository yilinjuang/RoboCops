#! /usr/bin/env python

import rospy
import apriltags_ros
from apriltags_ros.msg import AprilTagDetectionArray

class QRDetection:
    def callback(self,data):
        print(data)


    def detect(self):
        rospy.Subscriber("tag_detections",AprilTagDetectionArray,self.callback)
        rate=rospy.Rate(1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detection',anonymous=True)
    q = QRDetection()
    q.detect()
