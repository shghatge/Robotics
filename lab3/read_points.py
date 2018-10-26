#!/usr/bin/env python
from __future__ import division
import rospy
import math
import time
import tf
from scipy.spatial import ConvexHull
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import Twist,  Point,  Quaternion
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

def djks(graph, start, end):
	visited = []
	unvisited = graph.keys()
	distance = []
	path = [None]*len(graph.keys())
	
	for i in unvisited:
		distance.append(float('inf'))

	distance[start] = 0
	
	while len(unvisited):

		curr_node = None
		min_ = float('inf')
		
		for i in unvisited:
			if( distance[i] < min_):
				min_ = distance[i]
				curr_node = i
		
		visited.append(curr_node)
		unvisited.remove(curr_node)
		adj = graph[curr_node].keys()
		
		index = 0
		for i in adj:
			if( distance[i] > ( distance[visited[-1]] + graph[curr_node][i]) ):
				distance[i] = distance[visited[-1]] + graph[curr_node][i]
				path[i] = visited[-1]

		index += 1

	print("distances from djks")
	print(distance)
	
	return path


def distance (pt0, pt1):
	return math.sqrt( ( pt0[0] - pt1[0] ) ** 2 + ( pt0[1] - pt1[1] ) ** 2 )

def get_shortest_path(relevant_segments, start, end):
	# print(relevant_segments)

	graph = {}
	vertices = {}
	vertices[ (0, 0) ] = 0
	index = 1

	for i in relevant_segments:
		if not ( tuple(i[0]) in vertices):
			vertices [ tuple(i[0]) ] = index
			index += 1

		if not ( tuple(i[1]) in vertices):
			vertices [ tuple(i[1]) ] = index
			index += 1
	# print(vertices)

	for i in vertices:
		graph [vertices [i]] = {}

	for i in relevant_segments:
		#  i is a tuple of two lists where each list in it is a point
		# print("iterating "+str(vertices[tuple(i[0])]) + " "+str(vertices[tuple(i[1])]))
		graph [ vertices[tuple(i[0])] ] [ vertices[tuple(i[1])] ] = distance ( i[0], i[1] )


	# print( djks(graph, 0, 20) )
	print ("start and end pts are " + str(vertices [ tuple(start) ]) + " " + str(vertices [ tuple(end) ]))
	return vertices, djks(graph, vertices [ tuple(start) ], vertices [ tuple(end) ])

def get_path_in_points(path, vertices, start, end):

	point_path = []

	curr = vertices[end]
	start = vertices[start]
	
	while curr != start:

		for x,y in vertices.iteritems():
			if y == curr:
				point_path = [x] + point_path
				break

		curr = path[ curr ]
		
	point_path = [(0,0)] + point_path
	return point_path

def getDist(pt1, pt2):

	distance = math.sqrt( ( pt1[0] - pt2[0] ) ** 2 + ( pt1[1] - pt2[1] ) ** 2) 
	return distance

def getAngle(pt1, pt2):

	if( ( pt1[0] - pt2[0] ) == 0):
		angle = math.pi / 2
	else:
		angle = math.atan( ( pt1[1] - pt2[1] ) / ( pt1[0] - pt2[0] ) );

	return angle

def translate_robot(distance):
    global rate, ros_rate
    print("Translating the robot by "+str(distance))
    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    linear_duration = distance / 1.0

    print("Speed "+str(move_cmd.linear.x)+" dur "+str(linear_duration))
    # Move for a time to go the desired distance
    ticks = int(linear_duration * ros_rate)

    for t in range(ticks):
        cmd_vel.publish(move_cmd)
        rate.sleep()

	move_cmd = Twist()
    cmd_vel.publish(move_cmd)
    rospy.sleep(1)
    

def rotate_robot(angle):
	global rate, ros_rate
	print("Rotating by angle "+str(angle))
	move_cmd = Twist()
	move_cmd.angular.z =  1.0
	angular_duration = angle / 1.0

	print("Speed "+str(move_cmd.angular.z)+" dur "+str(angular_duration))
	# Move for a time to turn to the desired angle
	ticks = int(angular_duration * ros_rate)

	for t in range(ticks):
		cmd_vel.publish(move_cmd)
		rate.sleep()

	# Stop the robot 
	move_cmd = Twist()
	cmd_vel.publish(move_cmd)
    
def move_robot(points):

	total_dist = 0
	for i in range( len(points) - 1 ):
		angle = getAngle( points[i], points[ i+1 ] )
		rotate_robot(angle)
		dist = getDist(points[i], points[ i+1 ])
		total_dist += dist
		translate_robot(dist)
	print("dist travelled")
	print(str(total_dist))


marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=500)
rospy.init_node('convex_hull', anonymous = False)
rospy.sleep(0.5)
ros_rate = 50

################################
tf_listener  =  tf.TransformListener()
odom_frame  =  '/odom'

#Subscribe to scan and odomotry and publish cmd_vel

# cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 2)
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(ros_rate)

# try:
#     tf_listener.waitForTransform( odom_frame,  '/odom' ,  rospy.Time(),  rospy.Duration( 1.0 ))
#     base_frame  =  '/odom'
# except(tf.Exception,  tf.ConnectivityException,  tf.LookupException):
#     rospy.loginfo( "Cannot find transform between /odom and /base_link or /base_footprint" )
#     rospy.signal_shutdown( "tf Exception" )

################################

                            
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

relevant_segments = list()
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

start_pt = [0, 0]
goal_pt = [float (goal[0]) / 100.0, float (goal[1]) / 100.0]

vertices, path = get_shortest_path(relevant_segments, start_pt, goal_pt)

path_points = get_path_in_points(path, vertices, tuple(start_pt), tuple(goal_pt))

print(path_points)

move_robot(path_points)

rospy.sleep(1)