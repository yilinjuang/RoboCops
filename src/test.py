#! /usr/bin/env python

# Ref: http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html

import time

import rospy
from tf import TransformListener

import detection
import movement

rospy.init_node('test', anonymous=True)
tf = TransformListener()
detector = detection.Detection()
mover = movement.Movement()

while True:
    raw_input()

    pose = detector.detection_data.detections[0].pose
    frame_id = detector.detection_data.detections[0].pose.header.frame_id
    pos = detector.detection_data.detections[0].pose.pose.position
    ori = detector.detection_data.detections[0].pose.pose.orientation

    pose = tf.transformPose("/map", pose)
    print(str(pose))

    mover.move_to(pose.pose.position)
