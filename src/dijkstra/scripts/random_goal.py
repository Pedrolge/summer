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
from dijkstra.msg import DijkstraAction


class RandomGoal:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('send_waypoint', DijkstraAction)
		 #self.client.wait_for_server()


		#self.get_map_srv = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
		#self.top_map = self.get_map_srv('lg_june14').map

	def send_random_goal(self):
		print "Sending random goal"
		

if __name__ == '__main__':
	rospy.init_node('random_goal')

	node = RandomGoal ()
	
	print "Press Ctrl + C to exit node"
	try:
		while True:
			node.send_random_goal()
	except (KeyboardInterrupt):
		print '\n Terminated'


