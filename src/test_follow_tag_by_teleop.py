#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import detection

rospy.init_node('test', anonymous=True)
detector = detection.Detection()
rate = rospy.Rate(5)
cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
move_cmd = Twist()
X_MAX = 0.4
X_SCALE = 1.0
Z_MAX = 1.3
Z_SCALE = 0.6
SPEED = 0.8

while not rospy.is_shutdown():
    if not detector.detected:
        continue
    pos = detector.detection_data.detections[0].pose.pose.position
    move_cmd = Twist()
    # No rotate if tag is in the middle.
    if abs(pos.x) < 0.1:
        move_cmd.angular.z = 0.0
    else:
        move_cmd.angular.z = -1 * pos.x / X_MAX * X_SCALE * SPEED
    # Break if too close.
    if pos.z < 0.4:
        move_cmd.linear.x = 0.0
    else:
        move_cmd.linear.x = pos.z / Z_MAX * Z_SCALE * SPEED
    print(str(move_cmd))
    cmd_vel_pub.publish(move_cmd)
    rate.sleep()
