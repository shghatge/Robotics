from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import random, math


class RRT(object):

	def __init__(self, vertices, start, goal, step):
		self.vertices = vertices
		self.goal = goal
		self.start = start
		self.step = step
		self.edges = []
		self.nodes = []
		self.nodes_parent = []
		self.nodes.append(start)
		self.nodes_parent.append(-1)
		self.done = False

		self.load_edges(vertices)

	def load_edges(self, verts):
		num_verts = len(self.vertices)
		first = False
		last = False
		first_pt = []

		for i in range( num_verts - 1 ):

			if first == False:
				if verts[i][0] == 0 and verts[i][1] == 0:
					first = True
					first_pt = [ verts[i+1][0], verts[i+1][1] ]
					continue

			if first == True:
				if verts[i+1][0] == 0 and verts[i+1][1] == 0:
					self.edges.append( [ verts[i][0], verts[i][1], first_pt[0], first_pt[1] ] )
					first = False
					continue

			if first == True:
				self.edges.append( [ verts[i][0], verts[i][1], verts[i+1][0], verts[i+1][1] ] )


	def gen_rand(self):
		x = random.randint(0, 600)
		y = random.randint(0, 600)
		return ( x, y)

	def get_dist(self, pt1, pt2):
		return math.sqrt( (pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) + (pt1[1] - pt2[1]) * (pt1[1] - pt2[1])  )


	def find_closest_node(self, randQ):		
		min_dist = 9999999999
		min_index = -1

		for i in range( len (self.nodes) ):
			dist = self.get_dist(randQ, self.nodes[i])
			if dist < min_dist:
				min_dist = dist
				min_index = i

		return min_index, min_dist

	def collision_segment(self, p1,p2,p3,p4):
		s1 = [p2[0]-p1[0], p2[1]-p1[1]]
		s2 = [p4[0]-p3[0], p4[1]-p3[1]]
		div = (-s2[0]*s1[1] + s1[0]*s2[1])
		if(div == 0):
			return False
		s = (-1*s1[1]*(p1[0]-p3[0]) + s1[0] * (p1[1]-p3[1])) / div
		t = (s2[0]*(p1[1]-p3[1]) - s2[1] * (p1[0] - p3[0])) / div
		if(s >= 0 and s<=1 and t>=0 and t<=1):
			x = p1[0] + (t * s1[0])
			y = p1[1] + (t * s1[1])
			if(self.pointIsClose([x,y],p3) or self.pointIsClose([x,y],p4) ):
				return False
			return True
		return False

	def collision_detect(self, p1, p2):

		for i in range( len (self.edges) ):

			if self.collision_segment(p1, p2, self.edges[i][0:2], self.edges[i][2:]) == True:
				# print("Collision Detected between " + str([p1,p2])+" and "+str(self.edges[i]))
				return True		
		return False

	def is_goal_reached(self, pt):

		if ( self.get_dist(pt, self.goal) < self.step ) and self.collision_detect(pt, self.goal) == False:
			return True
		else:
			return False

	def isclose(self,a, b, rel_tol=1e-09, abs_tol=0.0):
		return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

	def pointIsClose(self, p1, p2):
		return self.isclose(p1[0],p2[0]) and self.isclose(p1[1],p2[1])

	def grow_to_randq(self, randQ):
		
		closest_node_index, dist = self.find_closest_node(randQ)
		if dist < 1:
			return
		closest_node = self.nodes[closest_node_index]

		nQ = [ randQ[0] - closest_node[0], randQ[1] - closest_node[1] ]
		nQ[0] = nQ[0] / dist
		nQ[1] = nQ[1] / dist

		if(self.get_dist(randQ, closest_node) < self.step):

			if self.collision_detect(randQ, closest_node) == True:
				return	
			
			plt.plot([closest_node[0], randQ[0]], [closest_node[1], randQ[1]], markersize = 3, color = 'xkcd:green')
			self.nodes_parent.append(len(self.nodes))
			self.nodes.append(randQ)
			
			if self.is_goal_reached(randQ):
				plt.plot([self.goal[0], randQ[0]], [self.goal[1], randQ[1]], markersize = 3, color = 'xkcd:green')
				self.nodes_parent.append(len(self.nodes))
				self.nodes.append(self.goal)
				self.done = True
				return

		else:

			new_node = [ 0, 0]
			new_node[0] = closest_node[0] + nQ[0] * self.step
			new_node[1] = closest_node[1] + nQ[1] * self.step
				
			while(True):

				if self.collision_detect(self.nodes[-1], new_node) == True:
					return

				plt.plot([self.nodes[-1][0], new_node[0]], [self.nodes[-1][1], new_node[1]], markersize = 3, color = 'xkcd:green')
				self.nodes_parent.append(len(self.nodes))
				self.nodes.append(new_node.copy())

				if self.is_goal_reached(self.nodes[-1]):
					plt.plot([self.goal[0], self.nodes[-1][0]], [self.goal[1], self.nodes[-1][1]], markersize = 3, color = 'xkcd:green')
					self.nodes_parent.append(len(self.nodes))
					self.nodes.append(self.goal)
					self.done = True
					return
				# print("append "+str(new_node))
				
				if(self.get_dist(randQ, new_node) < self.step):

					if self.collision_detect(self.nodes[-1], new_node) == True:
						return

					plt.plot([self.nodes[-1][0], new_node[0]], [self.nodes[-1][1], new_node[1]], markersize = 3, color = 'xkcd:green')
					self.nodes_parent.append(len(self.nodes))
					self.nodes.append(new_node.copy())

					if self.is_goal_reached(self.nodes[-1]):
						plt.plot([self.goal[0], self.nodes[-1][0]], [self.goal[1], self.nodes[-1][1]], markersize = 3, color = 'xkcd:green')
						self.nodes_parent.append(len(self.nodes))
						self.nodes.append(self.goal)
						self.done = True
						return
					# print("append "+str(new_node))
					break

				new_node[0] = new_node[0] + nQ[0] * self.step
				new_node[1] = new_node[1] + nQ[1] * self.step


	def grow_tree(self):

		i = 0
		while(i < 2000):
			randQ = self.gen_rand()
			# print(randQ)
			# print(self.start)
			# plt.plot(randQ[0], self.start[0], [randQ[1], self.start[1]], marker = 'o', color = 'xkcd:blue')	
			# print("going into rand "+str(i))
			self.grow_to_randq( randQ )
			# print("out of rand")
			# print(self.nodes)
			# print(self.nodes_parent)
			if self.done == True:
				break
			i += 1






		