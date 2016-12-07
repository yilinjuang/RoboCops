#! /usr/bin/env python

import rospy
import apriltags_ros
from apriltags_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage

class QRDetection:
    def callback(self, data):
        print(data.header.stamp)
        print(data.data[:10])

    def capture(self):
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        rospy.spin()

    def detect(self):
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detection',anonymous=True)
    q = QRDetection()
    #  q.detect()
    q.capture()
