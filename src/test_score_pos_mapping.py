#! /usr/bin/env python

import time

import rospy

import detection
import score

rospy.init_node('test', anonymous=True)
detector = detection.Detection()
scorer = score.Score()
f = open("data1","w+")

while not rospy.is_shutdown():
    raw_input()
    score = scorer.send(detector.captured_image, detector.camera_info)
    # amount = len(detector.detection_data)
    # print("=" * 20)
    # print(score)
    # print(amount)
    # print("=" * 20)
    f.write(str(score) + "\n")
    pos = detector.detection_data[0].pose.pose.position
    ori = detector.detection_data[0].pose.pose.orientation
    f.write(str(pos) + "\n")
    f.write(str(ori) + "\n\n")
    f.flush()
f.close()
