#! /usr/bin/env python

import apriltags_ros
import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, CompressedImage

class Detection:
    def __init__(self, mover):
        # Reference to mover.
        self.mover = mover

        self.detected = False
        self.captured_image = CompressedImage()
        self.camera_info = CameraInfo()
        self.detection_data = AprilTagDetectionArray()

        self.best_score = -1
        self.best_idx = -1
        self.best_position = Point()

        # Register listener for detection.
        rospy.wait_for_message("tag_detections", AprilTagDetectionArray)
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.get_detect_data)

        # Register listener for capture.
        rospy.wait_for_message("/camera/rgb/image_color/compressed", CompressedImage)
        rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.get_capture_data)

        # Register listener for camera info.
        rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo)
        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.get_camera_info)

    def estimate_score(self):
        """Estimate score of the detected tag |self.detection_data|.

        score = orient + dist + center

        """
        best_score = -1
        best_idx = -1
        best_position = Point()
        for idx, data in enumerate(self.detection_data):
            # TODO: use more accurate value range of score especially orient.
            pos = data.pose.pose.position
            orient = 5

            if pos.z < 0.5:
                dist = 10
            elif pos.z < 0.8:
                dist = 9
            elif pos.z < 1.1:
                dist = 8
            else:
                dist = 7

            if pos.x < 0.07:
                center = 10
            elif pos.x < 0.10:
                center = 9
            elif pos.x < 0.12:
                center = 8
            elif pos.x < 0.15:
                center = 7
            elif pos.x < 0.17:
                center = 6
            elif pos.x < 0.20:
                center = 5
            elif pos.x < 0.22:
                center = 4
            elif pos.x < 0.24:
                center = 3
            elif pos.x < 0.27:
                center = 2
            elif pos.x < 0.33:
                center = 1
            else:
                center = 0

            score = orient + dist + center
            if score > best_score:
                best_score = score
                best_idx = idx
                best_position = pos
        self.best_score = best_score
        self.best_idx = best_idx
        self.best_position = best_position

    def get_detect_data(self, data):
        """Callback function of subscriber.

        Args:
            data (AprilTagDetectionArray)

        """
        self.detected = False
        if data.detections:
            if len(data.detections) > 1:
                print("Detected! (multiple)!")
            else:
                print("Detected!")
            self.detected = True
            self.detection_data = data.detections
            self.estimate_score()
            self.mover.abort()

    def get_capture_data(self, data):
        """Callback function of subscriber.

        Args:
            data (CompressedImage)

        """
        #  print("Captured!")
        self.captured_image = data
        #  print(data.header.stamp)
        #  print(data.data[:10])

    def get_camera_info(self, data):
        """Callback function of subscriber.

        Args:
            data (CameraInfo)

        """
        #  print("Got camera info!")
        self.camera_info = data
        #  print(data)


if __name__ == '__main__':
    rospy.init_node('detection', anonymous=True)
    q = Detection()
    rospy.spin()
