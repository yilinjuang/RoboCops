#! /usr/bin/env python

import time

import rospy

import detection
import score

rospy.init_node('test', anonymous=True)
detector = detection.Detection()
scorer = score.Score()
f = open("data1","w+")

while True:
    raw_input()
    # if detector.detected:
    score = scorer.send(detector.captured_image, detector.camera_info)
    f.write(str(score) + "\n")
    pos = detector.detection_data.detections[0].pose.pose.position
    ori = detector.detection_data.detections[0].pose.pose.orientation
    f.write(str(pos) + "\n")
    f.write(str(ori) + "\n\n")
    f.flush()
# time.sleep(0.5)
f.close()
