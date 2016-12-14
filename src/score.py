#! /usr/bin/env python

import rospy
import rpc_game_client
from rpc_game_client.msg import Alive, Score, GameState
from rpc_game_client.srv import ClientScore, PlayerScore
from std_msgs.msg import Header

class Score:
    def __init__(self):
        rospy.wait_for_service('/rpc_score')
        self.scoren = rospy.ServiceProxy('/rpc_score', PlayerScore)
        self.header = Header()

    def send(self, image, info):
        self.header.stamp = rospy.Time.now()
        try:
            score = self.scoren(self.header, image, info)
        except rospy.ServiceException as e:
            print("Service blocked: " + str(e))
            return -1

        if not score:
            print("Sending failed! Try again.")
            return self.send(image, info)
        else:
            print("Sent!")
            print("Score: " + str(score))
            return score.score
