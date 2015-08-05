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
from dijkstra.msg import PositionInfo


def distance(point1, point2):
	return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2))

class Storage:
	def __init__(self, new_node, new_weight):
		self.node = new_node
		self.weight = new_weight


class Dijkstra(object):
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	waypoint = "WayPoint1"
	closest = ""
	e_pose = ""
	current_edge = ""
	next_node = ""
	goal = PoseStamped()
	is_at_goal = False
	robot2 = PositionInfo()


	caution_edges = ['WayPoint7_WayPoint8_WayPoint9_WayPoint10']

	path = []
	vert = dict()

	def __init__(self):
        #client = actionlib.SimpleActionClient(<what server we want>, <message type used>)
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.server = actionlib.SimpleActionServer('send_waypoint', DijkstraAction, execute_cb=self.navigate_goal, auto_start = False)
		self.client.wait_for_server()


		self.get_map_srv = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
		self.top_map = self.get_map_srv('lg_june14').map

		rospy.Subscriber('current_node', String, self.update_estimated_pose)
		rospy.Subscriber('closest_node', String, self.update_closest_node)
		rospy.Subscriber('robot2/position_info', PositionInfo, self.update_robot2_node)
		self.pub = rospy.Publisher('robot1/position_info', PositionInfo, queue_size=3)

		self.server.start()

	def send_random_goal (self):
				
		nodes = dict()

		goal_list = []

		for node in self.top_map.nodes:
			nodes[node.name] = node

		for name in nodes:
			goal_list.append(name)

		goal_list.remove("ChargingPoint")

		choice = randint(1, len(goal_list)-1)

		print "Goal list :" + str(goal_list)
		print goal_list[choice]

		path = self.make_plan(goal_list[choice])

		for i in range(1, len(path)):
			print "Next waypoint: %s" %(path[i])
			self.next_node  = path[i]
			pos = self.vert[path[i]]

			self.goal = PoseStamped()
		    #relative to map = map, relative to robot = base_link
			self.goal.header.frame_id = "map"
			self.goal.header.stamp = rospy.get_rostime()
		    
			self.goal.pose.position.x = pos.x
			self.goal.pose.position.y = pos.y
			self.goal.pose.orientation = Quaternion(0,0,0.931,0.365)
		    
		    #moveBaseAction takes a MoveBaseGoal which takes a PoseStamped
			self.client.send_goal(MoveBaseGoal(self.goal), feedback_cb=self.feedback)

			while not self.has_reached_goal():
				self.client.wait_for_result(rospy.Duration(1.0))
				if (self.client.get_state() == GoalStatus.PREEMPTED or self.client.get_state() == GoalStatus.ABORTED or self.client.get_state() == GoalStatus.LOST):
					fail = True

					break
			if (fail):
				break
		
		if fail:
			print "Goal could not be reached"			
		else:	
			print "Goal reached"

		return True


	def navigate_goal (self, goal):
		wp = goal.goal_waypoint

		path = self.make_plan(wp)
		fail = False

		for i in range(1, len(path)):
			print "Next waypoint: %s" %(path[i])


			self.next_node  = path[i]

			self.current_edge = self.e_pose + "_" + self.next_node
			self.publish_info()

			pos = self.vert[path[i]]

			self.goal = PoseStamped()
			self.goal.header.frame_id = "map"
			self.goal.header.stamp = rospy.get_rostime()
		    
			self.goal.pose.position.x = pos.x
			self.goal.pose.position.y = pos.y
			self.goal.pose.orientation = Quaternion(0,0,0.931,0.365)
		    
			self.client.send_goal(MoveBaseGoal(self.goal), feedback_cb=self.feedback)

			while not self.has_reached_goal():
				self.client.wait_for_result(rospy.Duration(1.0))
				if (self.client.get_state() == GoalStatus.PREEMPTED or self.client.get_state() == GoalStatus.ABORTED or self.client.get_state() == GoalStatus.LOST):
					fail = True

					break
			if (fail):
				break
		

		if fail:
			print "Goal could not be reached"
			self.server.set_aborted()		
		else:	
			print "Goal reached"
			self.server.set_succeeded()

	def safe_pos(self):
		print "calculating a safe position"

	def cancel_goal():
		client.cancel_goal()

	def update_estimated_pose(self, data):
		self.e_pose = data.data
		self.publish_info()
		#print "Updated current waypoint: %s" %self.e_pose

	def update_closest_node(self, data):
		self.closest = data.data
		self.publish_info()
		#print "Updated closest waypoint: %s" %self.closest

	def update_robot2_node(self, data)
		self.robot2 = data

	def publish_info(self):
		message = PositionInfo()
		message.closest_node = self.closest
		message.current_node = self.e_pose
		message.current_edge = self.current_edge

		self.pub.publish(message)


	def is_moving(self):
		return not(self.client.get_state() == GoalStatus.SUCCEEDED or self.client.get_state() == GoalStatus.ABORTED)

	def has_reached_goal(self):
		if (self.next_node == self.e_pose):
			return True
		else:
			return False

	def feedback(self,data):
		self.pose = data.base_position.pose
        #print str(self.pose.position.x) + ' ' + str(self.pose.position.y)
        #rospy.Duration.from_sec(5.0) to wait for 5 seconds
        

	def make_plan(self, goal):
		while (self.e_pose == "" or self.e_pose == "none"):
			if (self.closest != "" and self.closest != "none"):
				data = String()
				data.data = self.closest
				self.update_estimated_pose(data)
				print "Current node changed to closest node: %s" %self.closest
			rospy.sleep(1)



		init = self.e_pose
		end = goal
		
		nodes = dict()

		for node in self.top_map.nodes:
			nodes[node.name] = node

		self.vert = dict()
		edge = dict()
		visited = dict()
		unvisited = []

		#print "------------------------Getting nodes positions--------------------"
		#Defining the vertice's position
		for name, node in nodes.items():
			self.vert[name] = node.pose.position		
			#print "(x, y) = (" + str(self.vert[name].x) + ", " + str(self.vert[name].y) + ")"	

		#print "------------------------Getting edges weights----------------------"
		#Defining the weight of each edge by calculating the distance between vertices
		for name, node in nodes.items():
			for edge_temp in node.edges:
				edge[edge_temp.edge_id] = distance(self.vert[name], self.vert[edge_temp.node])
				#print "Distance between vertex " + name + " and " + edge_temp.node + " = " + str(edge[name+"_"+edge_temp.node])

		#print "------Setting the initial weights for the non visited nodes--------"
		#Define the visited vertices, and store the best previous vertex and the total weight to reach that vertex
		current = init
		for name, node in nodes.items():
			visited[name] = Storage("", 10000000)
			unvisited.append(name)
		
		visited[init] = Storage("", 0)

		#print unvisited

		#for name, weight in visited.items():
			#print "Calculated weightsfor nodes %s : (%s, %.2f)" % (name, weight.node, weight.weight)
	
		#Explore and update the weight to reach each node
		#print "------------------Updating map with new weights--------------------"
		while end in unvisited:
			min_name = ""
			min_value = 1000000000000
			for name in unvisited:
				if (visited[name].weight < min_value):
					min_value = visited[name].weight
					min_name = name

			#print "Vertex with minimum total weight: %s, %.2f" % (min_name, min_value)
	
			for edge_temp in nodes[min_name].edges:
				if (min_value + edge[edge_temp.edge_id]) < visited[edge_temp.node].weight:
					visited[edge_temp.node] = Storage(min_name, min_value + edge[edge_temp.edge_id])

			unvisited.remove(min_name)

	   		#for name, weight in visited.items():
				#print "Calculated weights for nodes %s : (%s, %.2f)" % (name, weight.node, weight.weight)

		node_name = end
		path = []
		path.append(node_name)
		while (node_name != init):
			path.append(visited[node_name].node)
			node_name = visited[node_name].node

		path = path[::-1]
		print "This is the path: " + str(path)
		return path


	def canGoTo(self, node):
		pass



if __name__ == '__main__':
	rospy.init_node('dijkstra_planner')

	node = Dijkstra()

	#node.send_random_goal()

	#while (True):
	#	while(not node.send_random_goal()):
	#		rospy.sleep(1)
	
	rospy.spin()







