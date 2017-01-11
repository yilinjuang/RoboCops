#! /usr/bin/env python

import math
import random
import sys
from timeit import default_timer as timer

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
        self.TO_SHOOT_OR_NOT_TO_SHOOT = 20
        self.cool_down = timer() - 15

    def run(self):
        pos = self.mover.get_position()
        print("Position: " + pos)

        if self.mover.disabled:
            # After getting shot!
            print("===AfterGettingShot===")
            # TODO
            return

        if not self.detector.detected:
            # Explore!
            print("===Explore===")
            self.mover.explore()
        else:
            if timer() - self.cool_down < 10:
                # After shooting!
                print("===CoolDown(Explore)===")
                self.mover.explore()
            else:
                if self.detector.best_score > self.TO_SHOOT_OR_NOT_TO_SHOOT:
                    # Shoot!
                    print("===Shoot===")
                    score = self.scorer.send(self.detector.captured_image,
                            self.detector.camera_info)
                    if score > 0:
                        # Shot successfully.
                        self.cool_down = timer()
                else:
                    # Follow!
                    print("===Follow===")
                    self.mover.follow(self.detector.best_position)

        self.rate.sleep()

    def shutdown(self):
        print("Shutdown!")

if __name__ == '__main__':
    try:
        main = Main()
        while not rospy.is_shutdown():
            main.run()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
