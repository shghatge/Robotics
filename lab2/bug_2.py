import rospy
import tf
import time
import math
from geometry_msgs.msg import Twist,  Point,  Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Callback function copies the distance of obstacle right in front and at the right most angle of view
def scan_callback(msg):
    # print("Inside LaserScan callback")
    global front_range, right_range 
    front_range = msg.ranges[ len( msg.ranges ) / 2 ]
    right_range = msg.ranges[-1]

#callback function to get the current position and pose of the robot

def get_odom():
    try:
        trans, rot = tf_listener.lookupTransform( odom_frame, base_frame, rospy.Time(0) )
    except  (tf.Exception,  tf.ConnectivityException,  tf.LookupException):
        rospy.loginfo( "TF Exception" )
        return
    return  Point(trans[0], trans[1], trans[2]), rot

# def odom_callback(odom_data):
#     # print("Inside Odometry callback")
#     global robot_pos, robot_pose 
#     robot_pos = odom_data.pose.pose.position
#     robot_pose = odom_data.pose.pose.orientation

def translate_robot(distance):
    global rate
    move_cmd.linear.x = 0.1
    linear_duration = distance / 0.1
    # Move for a time to go the desired distance
    ticks = int(linear_duration * rate)

    for t in range(ticks):
        self.cmd_vel.publish(move_cmd)
        rate.sleep()

    move_cmd = Twist()
    self.cmd_vel.publish(move_cmd)
    rospy.sleep(1) 

def rotate_robot(angle):
    global rate, angle_turned
    angle_turned += angle

    move_cmd.angular.z = 2
    angular_duration = angle / 2
    # Move for a time to turn to the desired angle
    ticks = int(angular_duration * rate)

    for t in range(ticks):
        self.cmd_vel.publish(move_cmd)
        rate.sleep()

    # Stop the robot 
    move_cmd = Twist()
    self.cmd_vel.publish(move_cmd)
    rospy.sleep(1)  

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
    val = True
    if ( (robot_pos.y < -0.1) or (robot_pos.y > 0.1) ):
        val = False
    if ( (robot_pos.z < -0.1) or (robot_pos.z > 0.1) ):
        val = False
    return val

def is_hit_point():
    val = False

    if ( abs(robot_pos.y - hit_point.y) <= 0.2 and abs(robot_pos.x - hit_point.x) <= 0.2):
        val = True

    return val

def isObstacleAhead():

    if( math.isnan(front_range) == True)
        return False

    if (front_range > 0.5)
        return False

    return True

def follow_wall():

    while( math.isnan(right_range) == False)
        rotate_robot(2)

    while ( is_m_line() == True or is_hit_point() == True):
        translate_robot(0.1)

        if( math.isnan(right_range) == False):
            rotate_robot(-2)
        else:
            rotate_robot(2)
            translate_robot(0.1)

    if (is_m_line() == True ):
        rotate_robot(-angle_turned)

    if (is_hit_point() == True):
        impossible = True






# These variables are updated as the robot moves, initialised to arbitrary values
front_range = 1.0
right_range = 0.8
hit_point = None
impossible = False

angle_turned = 0

# These variables store the position and pose of robot
#robot_pos = None
#robot_pose = None

tf_listener  =  tf.TransformListener()
odom_frame  =  '/odom'

#Subscribe to scan and odomotry and publish cmd_vel

scan_top = rospy.Subscriber('scan', LaserScan, scan_callback)
# odo_top = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 100)
rate = rospy.Rate(20)

rospy.init_node('bug_2', anonymous = False)

try:
    tf_listener.waitForTransform( odom_frame,  '/base_footprint' ,  rospy.Time(),  rospy.Duration( 1.0 ))
    base_frame  =  '/base_footprint'
except(tf.Exception,  tf.ConnectivityException,  tf.LookupException):
    rospy.loginfo( "Cannot find transform between /odom and /base_link or /base_footprint" )
    rospy.signal_shutdown( "tf Exception" )
    robot_pos, robot_pose = get_odom()
    
# This loop runs until the goal is reached
while (isGoalReached() == False):
    robot_pos, robot_pose = get_odom()

    if (isObstacleAhead() == False):
        translate_robot(0.1)
    else:
        hit_point = robot_pos
        hit_point.x = hit_point.x + front_range
        print("Obstacle Hit")
        follow_wall()

        if(impossible == True):
            print("Impossible to reach !")
            break
    