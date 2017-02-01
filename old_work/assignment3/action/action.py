#! /usr/bin/env python

import rospy
import actionlib
import move_base_msgs.msg

def hanging_around():
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    pos1 = move_base_msgs.msg.MoveBaseGoal()
    pos1.target_pose.pose.position.x = -2.0
    pos1.target_pose.pose.position.y = -1.0
    pos1.target_pose.pose.position.z = 0.0
    pos1.target_pose.pose.orientation.w = 1.0
    pos1.target_pose.header.frame_id = 'map'
    pos2 = move_base_msgs.msg.MoveBaseGoal()
    pos2.target_pose.pose.position.x = -2.0
    pos2.target_pose.pose.position.y = -2.0
    pos2.target_pose.pose.position.z = 0.0
    pos2.target_pose.pose.orientation.w = 1.0
    pos2.target_pose.header.frame_id = 'map'
    where_r_we_now = 1
    while True:
        print where_r_we_now
        if where_r_we_now == 1:
            pos2.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(pos2)
            where_r_we_now = 2
        else:
            pos1.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(pos1)
            where_r_we_now = 1
        client.wait_for_result()
        #  print "Result: " + client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('action')
        hanging_around()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
