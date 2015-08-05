#!/usr/bin/env python

import rospy
import actionlib
import time
    
from random import *
from math import *

from std_msgs.msg import String
from util import rotateQuaternion, getHeading

from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from strands_navigation_msgs.srv import GetTopologicalMap
from dijkstra.msg import DijkstraAction, DijkstraGoal


class RandomGoal:
	def __init__(self):
		self.feedback = ""

		self.client = actionlib.SimpleActionClient('send_waypoint', DijkstraAction)
		self.client.wait_for_server()

		self.get_map_srv = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
		self.top_map = self.get_map_srv('lg_coordination').map

	def send_random_goal(self):

		nodes = dict()

		goal_list = []

		for node in self.top_map.nodes:
			nodes[node.name] = node

		for name in nodes:
			goal_list.append(name)

		if ("ChargingPoint" in goal_list):
			goal_list.remove("ChargingPoint")

		choice = randint(1, len(goal_list)-1)

		print "Goint to ", goal_list[choice]
		
		
		self.client.send_goal(DijkstraGoal(goal_list[choice]), feedback_cb=self.feedback)
		self.client.wait_for_result()		

if __name__ == '__main__':
	rospy.init_node('random_goal')

	print "Press Ctrl + C to exit node"
	try:
		node = RandomGoal ()
		while True:
			node.send_random_goal()
			time.sleep(1)
	except (KeyboardInterrupt, SystemExit):
		print "This was called again"


	rospy.spin()

