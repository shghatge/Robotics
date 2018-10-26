#!/usr/bin/env python
from __future__ import division
import rospy
from scipy.spatial import ConvexHull
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import itertools
from collections import defaultdict
def load_obstacles(object_path):
	'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
	'''
	obstacles = []
	obstacle = []
	with open(object_path) as f:
		numObstacles = int(f.readline())
		coordinates = int(f.readline())
		for i in range(coordinates):
			line = f.readline()
			obstacle.append(list(map(int, line.strip().split(' '))))
		for line in f:
			coordinates = list(map(int, line.strip().split(' ')))
			if len(coordinates) == 1:
				obstacles.append(obstacle)
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
	return obstacles

def load_goal(object_path):
	with open(object_path) as f:
		line = f.readline()
		return line.strip().split(' ')

def pub_path(pointsInput,hull):
    points = Marker()
    line_strips = Marker()
    points.header.frame_id="base_link"
    line_strips.header.frame_id="base_link"
    points.ns="points"+hull
    line_strips.ns="linestrips"+hull
    points.id=1;
    line_strips.id=2
    points.type = Marker.POINTS;
    line_strips.type = Marker.LINE_STRIP;
    for p in pointsInput:
    	points.points.append(p)
    	line_strips.points.append(p)
    line_strips.points.append(pointsInput[0])
    points.pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    points.scale=Vector3(0.03, 0.03, 0.03)
    points.color=ColorRGBA(0.0, 1.0, 0.0, 0.8)
    line_strips.pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    line_strips.scale=Vector3(0.03, 0.03, 0.03)
    line_strips.color=ColorRGBA(1.0, 0.0, 0.0, 0.8)  
    points.lifetime = rospy.Duration()
    line_strips.lifetime = rospy.Duration()
        
        # publish points
    marker_publisher.publish(points)
    marker_publisher.publish(line_strips)

def publish_lines(all_segments):
    line_strips = Marker()
    line_strips.header.frame_id="base_link"
    line_strips.ns="segments"
    line_strips.id=3
    line_strips.type = Marker.LINE_STRIP;
    line_strips.pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    line_strips.scale=Vector3(0.03, 0.03, 0.03)
    line_strips.color=ColorRGBA(1.0, 0.0, 0.0, 0.8)  
    ns = 0
    for s in all_segments:
    	line_strips.ns="segment"+str(ns)
    	ns = ns+1
    	line_strips.points=[]
    	line_strips.points.append(Point(s[0][0], s[0][1], 0))
     	line_strips.points.append(Point(s[1][0], s[1][1], 0))  	
    	line_strips.lifetime = rospy.Duration()
    	marker_publisher.publish(line_strips)
    	rospy.sleep(0.005)
    print(ns)  
        # publish points

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def pointIsClose(p1, p2):
	return isclose(p1[0],p2[0]) and isclose(p1[1],p2[1])

def collision_detect(p1,p2,p3,p4):
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
		if(pointIsClose([x,y],p3) or pointIsClose([x,y],p4) ):
			return False
		return True
	return False

marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=500)
# if __name__ == "__main__":

rospy.init_node('convex_hull', anonymous = False)
rospy.sleep(0.5)                                                             
obstacles = load_obstacles("../data/world_obstacles.txt")
goal = load_goal("../data/goal.txt")
print("goal", goal)
grown_obstacles = []
i=0;
allpoints = list()
allpoints.append(goal)
allpoints.append([0,0])
for obs in obstacles:
	grown_obstacle = []
	points = []
	for vertex in obs:
		points.append(Point(vertex[0]/100,vertex[1]/100,0))
		xx = [vertex[0]+18, vertex[1]+18]
		xy = [vertex[0]+18, vertex[1]-18]
		yx = [vertex[0]-18, vertex[1]+18]
		yy = [vertex[0]-18, vertex[1]-18]
		grown_obstacle.append(xx)
		grown_obstacle.append(xy)
		grown_obstacle.append(yx)
		grown_obstacle.append(yy)
	grown_obstacle_array = np.array(grown_obstacle)
	ch = ConvexHull(grown_obstacle_array)
	grown_obstacles.append([ch, grown_obstacle_array])

edges_hull = list()
vertices_hull = defaultdict(list)
for obstacle in grown_obstacles:
	points = []
	pts = []
	for vertex in obstacle[0].vertices: 
		vertices_hull[i].append([obstacle[1][vertex][0],obstacle[1][vertex][1]])
		points.append(Point(obstacle[1][vertex][0]/100,obstacle[1][vertex][1]/100,0))
		pts.append([obstacle[1][vertex][0]/100,obstacle[1][vertex][1]/100])
	pub_path(points,str(i))  
	l = len(points)
	i = i+1
	for j in range(0,l):
		edges_hull.append((pts[j],pts[(j+1)%(l)]))
rospy.sleep(1)  
all_segments = []
all_segments.append(([0,0],[int(goal[0])/100,int(goal[1])/100]))
for k in range(0 , len(obstacles)):
	for x in vertices_hull[k]:
		all_segments.append(([0,0],[x[0]/100,x[1]/100]))
	for x in vertices_hull[k]:
		all_segments.append(([x[0]/100,x[1]/100],[int(goal[0])/100,int(goal[1])/100]))
	for x in vertices_hull[k]:
		for t in range(0, len(obstacles)):
			if k != t :
				for v in vertices_hull[t]:
					all_segments.append(([x[0]/100,x[1]/100],[v[0]/100,v[1]/100]))

relevant_segments = []
for seg in all_segments:
	add = True
	for edge in edges_hull:
		if collision_detect(seg[0], seg[1], edge[0], edge[1]) == True :
			add = False
	if add :
		relevant_segments.append(seg)
for edge in edges_hull:
	relevant_segments.append(edge)
publish_lines(relevant_segments)
rospy.sleep(1)  