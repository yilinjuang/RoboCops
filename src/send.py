#! /usr/bin/env python

import game_client
import roslib
import rospy

def sendin
  	rospy.wait_for_service('rpc_score')
	scoren = rospy.serviceProxy('rpc_score',

if __name__ == '__main__':
    try:
        send_in = sendin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
