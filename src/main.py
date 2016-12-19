#! /usr/bin/env python

import math
import random
import sys

import rospy

import detection
import movement
import score

class Main:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('robocops')

        self.mover = movement.Movement()
        self.detector = detection.Detection()
        self.scorer = score.Score()

        self.rate = rospy.Rate(10)

    def action(self):
        # goal = raw_input()
        # self.mover.explore()
        # self.mover.move_to(goal)
        pos = self.mover.get_position()
        print(pos)

        self.rate.sleep()

    def shutdown(self):
        print("Shutdown!")

if __name__ == '__main__':
    try:
        main = Main()
        while not rospy.is_shutdown():
            main.action()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
