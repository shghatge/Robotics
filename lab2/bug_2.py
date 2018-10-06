#!/usr/bin/env python

import rospy
import tf
import time
import math
from geometry_msgs.msg import Twist,  Point,  Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle


def get_min_range_right():
    global ranges
    distance_min=9999
    for i in range( 320):
        if (distance_min>ranges[i]):
            distance_min=ranges[i]
    return (distance_min)

'''def get_min_range_right():
    global ranges
    distance_min=9999
    for i in range(len(ranges)/2):
        if (distance_min>ranges[640-i-1]):
            distance_min=ranges[640-i-1]
               
    return (distance_min)'''

# Callback function copies the distance of obstacle right in front and at the right most angle of view
def scan_callback(msg):
    # print("Inside LaserScan callback")
    global front_range, right_range 
    front_range = msg.ranges[ len( msg.ranges ) / 2 ]
    right_range = msg.ranges[-1]
    global ranges
    ranges = msg.ranges
    right_range = get_min_range_right()
    
#callback function to get the current position and pose of the robot



def get_odom_pos():
    try:
        trans, rot = tf_listener.lookupTransform( odom_frame, base_frame, rospy.Time(0) )
    except  (tf.Exception,  tf.ConnectivityException,  tf.LookupException):
        rospy.loginfo( "TF Exception" )
        return

    print("Positions "+str(trans[0])+" "+str(trans[1]))
    return Point(trans[0], trans[1], trans[2])

def get_odom_pose():
    try:
        trans, rot = tf_listener.lookupTransform( odom_frame, base_frame, rospy.Time(0) )
    except  (tf.Exception,  tf.ConnectivityException,  tf.LookupException):
        rospy.loginfo( "TF Exception" )
        return

    print("Poses "+str(quat_to_angle(Quaternion(*rot))))
    return quat_to_angle(Quaternion(*rot))

# def odom_callback(odom_data):
#     # print("Inside Odometry callback")
#     global robot_pos, robot_pose 
#     robot_pos = odom_data.pose.pose.position
#     robot_pose = odom_data.pose.pose.orientation

def translate_robot(distance):
    global rate, robot_pos, robot_pose

    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    linear_duration = distance / 0.2
    # Move for a time to go the desired distance
    ticks = int(linear_duration * ros_rate)

    for t in range(ticks):
        cmd_vel.publish(move_cmd)
        rate.sleep()

    move_cmd = Twist()
    cmd_vel.publish(move_cmd)
    rospy.sleep(1) 
    robot_pos = get_odom_pos()
    print(robot_pos)

def rotate_robot(angle, direction):
    global rate, angle_turned, robot_pose

    #angle_turned += ((direction) * angle)
    print("Rotating by angle "+str(angle))
    move_cmd = Twist()
    move_cmd.angular.z = direction * 1
    angular_duration = angle / 1
    # Move for a time to turn to the desired angle
    ticks = int(angular_duration * ros_rate)

    for t in range(ticks):
        cmd_vel.publish(move_cmd)
        rate.sleep()

    # Stop the robot 
    move_cmd = Twist()
    cmd_vel.publish(move_cmd)
    robot_pose = get_odom_pose()
    print(robot_pose)

def isGoalReached():
    val = True
    if ( (robot_pos.x < 9.9) or (robot_pos.x > 10.1) ):
        val = False
    if ( (robot_pos.y < -0.1) or (robot_pos.y > 0.1) ):
        val = False
    if ( (robot_pos.z < -0.1) or (robot_pos.z > 0.1) ):
        val = False
    return val

def is_m_line():
    val = False
    print("M_Line Check : Pos "+str(robot_pos.x)+" "+str(robot_pos.y))

    if ( (robot_pos.y >= -0.1) and (robot_pos.y <= 0.1) ):
	if(robot_pos.x > hit_point.x):
	    val = True
    	    print("M_Line Check is Hit: Pos "+str(robot_pos.x)+" "+str(robot_pos.y))
    
    return val

def is_hit_point():
    val = False

    if ( abs(robot_pos.y - hit_point.y) <= 0.2 and abs(robot_pos.x - hit_point.x) < 0.05):
        val = True
	print("HitPoint Check is Hit: Pos "+str(robot_pos.x)+" "+str(robot_pos.y))

    return val

def isObstacleAhead():

    if( math.isnan(front_range) == True):
        return False

    if (front_range > 1.0):
        return False

    return True

def follow_wall():
    global impossible
    print("Following wall")

    while( right_range < 9999):
        print(" rotate until right is NAN Ranges "+str(front_range) +" "+str(right_range))
        rotate_robot(1, 1)

    while ( True ):
        
        print(" Move forward Ranges "+str(front_range) +" "+str(right_range))
	translate_robot(translate_speed)

        if( right_range > 3.0):
            rotate_robot(1, -1)

        if( right_range < 0.8):
            rotate_robot(1, 1)

        if (is_m_line() == True ):
	    print("M line is hit, should turn "+str(robot_pose))
	    if(robot_pose < 0):
                rotate_robot(abs(robot_pose), 1)
	    else:
		rotate_robot(abs(robot_pose), -1)
	    break

        if (is_hit_point() == True):
            impossible = True
	    break





# These variables are updated as the robot moves, initialised to arbitrary values
front_range = 9999
right_range = 9999
translate_speed = 0.2
hit_point = None
impossible = False
robot_pose = 0.0
robot_pos = Point(0.0, 0.0, 0.0)

ros_rate = 50

angle_turned = 0

# These variables store the position and pose of robot
#robot_pos = None
#robot_pose = None

rospy.init_node('bug_2', anonymous = False)
tf_listener  =  tf.TransformListener()
odom_frame  =  '/odom'

#Subscribe to scan and odomotry and publish cmd_vel

scan_top = rospy.Subscriber('scan', LaserScan, scan_callback)
# odo_top = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 2)
rate = rospy.Rate(ros_rate)

try:
    tf_listener.waitForTransform( odom_frame,  '/base_footprint' ,  rospy.Time(),  rospy.Duration( 1.0 ))
    base_frame  =  '/base_footprint'
except(tf.Exception,  tf.ConnectivityException,  tf.LookupException):
    rospy.loginfo( "Cannot find transform between /odom and /base_link or /base_footprint" )
    rospy.signal_shutdown( "tf Exception" )
    #robot_pos, robot_pose = get_odom()
    
# This loop runs until the goal is reached
while (isGoalReached() == False):
    print("Goal not reached")
    #robot_pos, robot_pose = get_odom()

    if (isObstacleAhead() == False):
        print(" Ranges "+str(front_range) +" "+str(right_range))
	print("Obstace not ahead")
        translate_robot(translate_speed)
    else:
        hit_point = robot_pos
        hit_point.x = hit_point.x + front_range
        print(" Ranges "+str(front_range) +" "+str(right_range))
        print("Obstacle Hit, Hit Point "+str(hit_point.x)+" "+str(hit_point.y))
        follow_wall()

        if(impossible == True):
            print("Impossible to reach !")
            break
    
