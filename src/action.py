#! /usr/bin/env python

import math
import random
import sys

import detection
import actionlib
import move_base_msgs.msg
import rospy
import tf
from geometry_msgs.msg import Point, TransformStamped
from collections import deque

class Map:
    def __init__(self):
        self.locations = ["D1", "D2", "D3", "D4", "D5", "B1", "B2", "B3", "B4"]
        self.points = {"D1": Point(-3.3698, -1.7438, 0),
                          "D2": Point(-1.7187, -6.6091, 0),
                          "D3": Point(-0.93385, -11.325, 0),
                          "D4": Point(0.79652, -15.72, 0),
                          "D5": Point(2.4906, -20.594, 0),
                          "B1": Point(2.2752, 0.74912, 0),
                          "B2": Point(-5.7483, 1.7007, 0),
                          "B3": Point(-6.0922, -5.5226, 0),
                          "B4": Point(2.0646, -22.999, 0)}

    def get_distance(pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

class Movement:
    def __init__(self):
        self.map_ = Map()
        self.tf = tf.TransformListener()

        # Last n visited locations
        self.visited = deque(self.map_.locations, 3)

        # MoveBaseGoal
        self.pos = move_base_msgs.msg.MoveBaseGoal()
        self.pos.target_pose.pose.orientation.w = 1.0
        self.pos.target_pose.header.frame_id = 'map'

        # SimpleActionClient
        self.client = actionlib.SimpleActionClient('move_base',
                move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()

    # goal: key of Map.points
    # return false if failed to move to goal
    def move_to(self, goal):
        self.pos.target_pose.pose.position = self.map_.points[goal]
        self.pos.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.pos)
        return self.client.wait_for_result(rospy.Duration(10))

    def explore(self):
        goal = random.choice(self.map_.locations)
        while goal in self.visited:
            goal = random.choice(self.map_.locations)
        print("Explore: " + str(goal))
        success = self.move_to(goal)
        if not success:
            self.client.cancel_goal()
            print("Failed to explore " + str(goal))
        else:
            self.visited.append(goal)

    def get_position(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            now = rospy.Time.now()
            self.tf.waitForTransform("/base_link", "/map", now,
                    rospy.Duration(4.0))
            position, rotation = self.tf.lookupTransform("/base_link", "/map",
                    now)
            return position
        else:
            print("Frame doesn't exist.")

class Main:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('moving')
        self.movement = Movement()
        detector = detection.QRDetection()


    def action(self):
        #  goal = raw_input()
        self.movement.explore()
        #  self.movement.move_to(goal)
        #  pos = self.movement.get_position()

    def shutdown(self):
        print("Shutdown!")
        sys.exit()

if __name__ == '__main__':
    try:
        main = Main()
        while True:
            main.action()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
