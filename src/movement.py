#! /usr/bin/env python

import random
import time
from collections import deque

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, TransformStamped, Twist
from kobuki_msgs.msg import MotorPower
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf import TransformListener

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
    def __init__(self, main=None):
        self.main = main

        self.map_ = Map()
        self.tf = TransformListener()

        # Last n visited locations
        self.visited = deque(self.map_.locations, 3)

        # MoveBaseGoal
        self.pos = MoveBaseGoal()
        self.pos.target_pose.pose.orientation.w = 1.0
        self.pos.target_pose.header.frame_id = '/map'

        # SimpleActionClient
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.longest_waiting_time = rospy.Duration(10)

        # Clear costmaps
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps',
                Empty)

        # Register listener for motor availability and odometry.
        self.disabled = False
        rospy.wait_for_message("/mobile_base/commands/motor_power", MotorPower)
        rospy.Subscriber("/mobile_base/commands/motor_power", MotorPower,
                self._get_motor_info)
        self.odom = 0.0
        rospy.wait_for_message("/odom", Odometry)
        rospy.Subscriber('/odom', Odometry, self._get_odom_data)

        # Teleop
        self.teleop = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,
                queue_size=1)
        self.TELEOP_X_MAX = 0.4
        self.TELEOP_X_SCALE = 1.0
        self.TELEOP_Z_MAX = 1.3
        self.TELEOP_Z_SCALE = 0.6
        self.TELEOP_SPEED = 0.8

    def abort(self):
        self.client.cancel_all_goals()

    # goal: Point()
    def _move_to(self, goal):
        self.clear_costmaps()
        self.pos.target_pose.pose.position = goal
        self.pos.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.pos)

    # portal: key of Map.points
    def _move_to_portal(self, portal):
        goal = self.map_.points[portal]
        self._move_to(goal)

    def explore(self):
        portal = random.choice(self.map_.locations)
        while portal in self.visited:
            portal = random.choice(self.map_.locations)
        print("Explore: " + portal)
        self._move_to_portal(portal)
        while self.client.get_state() == GoalStatus.PENDING or \
                self.client.get_state() == GoalStatus.ACTIVE:
            time.sleep(0.05)
        if self.client.get_state() != GoalStatus.SUCCEEDED:
            print("Failed to explore " + portal)
        else:
            self.visited.append(portal)
            # Do a 360-degree rotation.
            teleop_cmd = Twist()
            teleop_cmd.angular.z = 2.0
            self.teleop.publish(teleop_cmd)

    def follow(self, pos):
        teleop_cmd = Twist()
        # No rotate if tag is in the middle.
        if abs(pos.x) < 0.1:
            teleop_cmd.angular.z = 0.0
        else:
            teleop_cmd.angular.z = -1 * pos.x / self.TELEOP_X_MAX * \
                    self.TELEOP_X_SCALE * self.TELEOP_SPEED
        # Break if too close.
        if pos.z < 0.4:
            teleop_cmd.linear.x = 0.0
        else:
            teleop_cmd.linear.x = pos.z / self.TELEOP_Z_MAX * \
                    self.TELEOP_Z_SCALE * self.TELEOP_SPEED
        print("Follow: distance = " + str(pos.z))
        self.teleop.publish(teleop_cmd)

    def rotate_to(self, target, direction):
        print("Now at: " + str(self.odom) + ". Rotate to: " + str(target))
        teleop_cmd = Twist()
        teleop_cmd.angular.z = direction * 0.5
        while not self.main.detector.detected and abs(self.odom - target) > 0.01:
            time.sleep(0.2)
            self.teleop.publish(teleop_cmd)

    def get_position(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            t = rospy.Time(0)
            self.tf.waitForTransform("/base_link", "/map", t,
                    rospy.Duration(4.0))
            position, rotation = self.tf.lookupTransform("/base_link", "/map",
                    t)
            return position
        else:
            print("Frame doesn't exist.")

    def _get_motor_info(self, data):
        """Callback function of subscriber.

        Args:
            data (MotorPower)

        """
        # print("Got motor info!")
        if data.state == MotorPower.ON:
            self.disabled = False
        else:
            self.disabled = True
        # print(data)

    def _get_odom_data(self, data):
        self.odom = data.pose.pose.orientation.z
        # print(self.odom)


if __name__ == '__main__':
    rospy.init_node('movement', anonymous=True)
    m = Movement()
    rospy.spin()
