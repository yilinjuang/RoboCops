#! /usr/bin/env python

import math

import rospy
import actionlib
import move_base_msgs.msg
from geometry_msgs.msg import Point

class Map:
    def __init__(self):
        self.locations = {"D1": Point(-3.3698, -1.7438, 0),
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

        self.pos = move_base_msgs.msg.MoveBaseGoal()
        self.pos.target_pose.pose.orientation.w = 1.0
        self.pos.target_pose.header.frame_id = 'map'

        self.client = actionlib.SimpleActionClient('move_base',
                move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()

    # goal: key of Map.locations
    def move_to(self, goal):
        self.pos.target_pose.pose.position = self.map_.locations[goal]
        self.pos.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.pos)
        self.client.wait_for_result()

    def explore(self):
        while True:
            for location in self.map_.locations:
                raw_input()
                print(location)
                self.move_to(location)


if __name__ == '__main__':
    try:
        rospy.init_node('moving')
        movement = Movement()
        movement.explore()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
