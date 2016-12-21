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
    # raw_input()
    time.sleep(0.2)

    if not detector.detected:
        continue

    pose = detector.detection_data.detections[0].pose
    pose.pose.position.z -= 0.5
    # frame_id = detector.detection_data.detections[0].pose.header.frame_id
    # pos = detector.detection_data.detections[0].pose.pose.position
    # ori = detector.detection_data.detections[0].pose.pose.orientation

    print(str(pose))
    t = rospy.Time(0)
    tf.waitForTransform("/camera_rgb_optical_frame", "/map", t, rospy.Duration(4.0))
    pose = tf.transformPose("/map", pose)

    mover.move_to(pose.pose.position)
