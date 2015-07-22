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
	visited = []
	goal = PoseStamped()
	is_at_goal = False
	robot2 = PositionInfo()


	#caution_edges = ['WayPoint7_WayPoint8_WayPoint9_WayPoint10', 'WayPoint28_WayPoint26_WayPoint21']

	path = []
	vert = dict()

	def __init__(self):
	
		self.my_name = rospy.get_param('my_name', 'robot1')
		self.other_robots = rospy.get_param('other_robots', ['robot2'])
		self.caution_edges = rospy.get_param('caution_edges', [])
		print self.my_name
		print self.other_robots
		print self.caution_edges

	        #client = actionlib.SimpleActionClient(<what server we want>, <message type used>)
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.server = actionlib.SimpleActionServer('send_waypoint', DijkstraAction, execute_cb=self.navigate_goal2, auto_start = False)
		self.client.wait_for_server()


		self.get_map_srv = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
		self.top_map = self.get_map_srv('lg_june14').map

		self.pub = rospy.Publisher(self.my_name + '/position_info', PositionInfo, queue_size=3)
		rospy.Subscriber('current_node', String, self.update_estimated_pose)
		rospy.Subscriber('closest_node', String, self.update_closest_node)

		if (len(self.other_robots) == 1)
		rospy.Subscriber(self.other_robots[0] + '/position_info', PositionInfo, self.update_robot2_node)


		self.server.start()
		print "Path planner server started. Waiting for goal..."

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


			self.next_node = path[i]
			#self.next_node = self.nextWP(path)

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


	def nextWP(self, path):

		next = ''


		#DEFINING NEXT WAYPOINT, BASED ON THE VISITED NODES OF THE PATH
		if len(self.visited) > 0:
			#print "Next Waypoint is: %s" %path[len(self.visited)]
			next = path[len(self.visited)]
		else:
			#print "Next Waypoint is: %s" %path[0]
			next = path[0]

	
		caution = []		
		for i in self.caution_edges:
			temp = i.split('_')
			caution.append(temp)

		m_intersections = 0
		m_caution = []
		m_common_nodes = []

		#CHECKING IF NEXT NODE IS IN ANY OF THE DANGEROUS PATHS
		safe = True
		for i in caution:
			if next in i:
				safe = False
				m_caution = i

		if safe:
			print "Next Waypoint is: %s" %next
			return next


		#CHECKING IF MY PATH INTERSECTS WITH ANY PATH THAT I NEED CAUTION
		for i in caution:
			n = 0
			common = []
			for j in path:
				if j in i:
					common.append(j)
					n += 1
			if (n > m_intersections):
				m_common_nodes = common
				m_intersections = n
				
		print "Number of intersections: %i" %m_intersections
		print "Nodes of my path in common with the caution path: " + str (m_common_nodes)


		##IF THE NUMBER OF NODES IN COMMON WITH A CAUTION PATH IS LESS THAN 2, THE PATHS DO NOT INTERSECT
		if m_intersections < 2:
			print "Next Waypoint is: %s" %next
			return next
		##
		#


	
		#CHECKING IF THE OTHER ROBOT IS INSIDE THE CAUTION PATH THAT I'M ENTERING
		robot2_edge = self.robot2.current_edge.split('_')

		o_intersections = 0
		o_common_nodes = []

		for i in robot2_edge:
			if i in m_caution:
				o_intersections += 1
				o_common_nodes.append(i)

		print "Number of intersections of the other robot's path: %i" %o_intersections
		print "Nodes of the other robot's path in common with the caution path: " + str (o_common_nodes)

		if (o_intersections < 2):
			print "Next Waypoint is: %s" %next
			return next



		#CALCULATING MY DIRECTION IN THE CAUTION PATH
		m_direction = 0
		
		if len(m_common_nodes) >= 2:
			a = m_common_nodes[0]
			b = m_common_nodes[1]
			m_direction = m_caution.index(b) - m_caution.index(a)

		print "My direction is: %i" %m_direction



		#CALCULATING THE DIRECTION OF THE OTHER ROBOT IN THE PATH
		o_direction = 0

		if len(o_common_nodes) == 2:
			a = o_common_nodes[0]
			b = o_common_nodes[1]
			o_direction = m_caution.index(b) - m_caution.index(a)

		print "Other robot's direction is: %i" %o_direction



		#CHECKING IF WE ARE GOING ON OPPOSITE DIRECTIONS

		if (m_direction * o_direction < 0):
			print "Waiting for the other robot to leave my path"
			next = self.e_pose
			rospy.sleep(10)

		print "Next Waypoint is: %s" %next
		return next



	def navigate_goal2 (self, goal):
		wp = goal.goal_waypoint

		path = self.make_plan(wp)



		fail = False
		self.visited = []

		while (self.e_pose != wp):

			#self.next_node = path[i]
			self.next_node = self.nextWP(path)

			self.current_edge = self.e_pose + "_" + self.next_node
			self.publish_info()

			pos = self.vert[self.next_node]

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
			else:
				if (self.next_node in path) and (self.next_node not in self.visited):
					self.visited.append(self.next_node)
					print "List of visited nodes updated"
					print self.visited
		

		if fail:
			print "Goal could not be reached"
			self.server.set_aborted()		
		else:	
			print "Goal reached"
			self.server.set_succeeded()


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

	def update_robot2_node(self, data):
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
			if visited[node_name].node != init:
				path.append(visited[node_name].node)
				node_name = visited[node_name].node
			else:
				node_name = visited[node_name].node

		path = path[::-1]
		print "This is the path: " + str(path)
		return path





if __name__ == '__main__':
	rospy.init_node('dijkstra_planner')


	node = Dijkstra()	
	rospy.spin()







