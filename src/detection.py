#! /usr/bin/env python

import apriltags_ros
import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo, CompressedImage

class Detection:
    def __init__(self):
        self.detected = False
        self.captured_image = CompressedImage()
        self.camera_info = CameraInfo()
        self.detection_data = AprilTagDetectionArray()

        self.detect()
        self.capture()
        self.info()

    def get_detect_data(self, data):
        """Callback function of detect().

        Args:
            data (AprilTagDetectionArray)

        """
        self.detected = False
        if data.detections:
            # TODO: multiple detections
            if len(data.detections) > 1:
                print("Multiple detections!")

            print("Detected!")
            self.detected = True
            self.detection_data = data
            pos = data.detections[0].pose.pose.position
            ori = data.detections[0].pose.pose.orientation
            # print(str(pos))
            # print(str(ori))

    def get_capture_data(self, data):
        """Callback function of capture().

        Args:
            data (CompressedImage)

        """
        #  print("Captured!")
        self.captured_image = data
        #  print(data.header.stamp)
        #  print(data.data[:10])

    def get_camera_info(self, data):
        """Callback function of info().

        Args:
            data (CameraInfo)

        """
        #  print("Got camera info!")
        self.camera_info = data
        #  print(data)

    def detect(self):
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.get_detect_data)

    def capture(self):
        rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.get_capture_data)

    def info(self):
        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.get_camera_info)

if __name__ == '__main__':
    rospy.init_node('detection', anonymous=True)
    q = Detection()
    rospy.spin()
