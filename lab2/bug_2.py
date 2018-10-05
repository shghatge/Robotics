import rospy
import tf
import time
from geometry_msgs.msg import Twist,  Point,  Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Callback function copies the distance of obstacle right in front and at the right most angle of view
def scan_callback(msg):
    print("Inside LaserScan callback")
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

def odom_callback(odom_data):
    print("Inside Odometry callback")
    global robot_pos, robot_pose 
    robot_pos = odom_data.pose.pose.position
    robot_pose = odom_data.pose.pose.orientation

def isGoalReached():
    val = True
    if ( (robot_pos.x < 9.9) or (robot_pos.x > 10.1) ):
        val = False
    if ( (robot_pos.y < -0.1) or (robot_pos.y > 0.1) ):
        val = False
    if ( (robot_pos.z < -0.1) or (robot_pos.z > 0.1) ):
        val = False
    return val

# These variables are updated as the robot moves, initialised to arbitrary values
front_range = 1.0
right_range = 0.8

# These variables store the position and pose of robot
#robot_pos = None
#robot_pose = None

tf_listener  =  tf.TransformListener()
odom_frame  =  '/odom'

#Subscribe to scan and odomotry and publish cmd_vel

scan_top = rospy.Subscriber('scan', LaserScan, scan_callback)
odo_top = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

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
    print("Position "+str(robot_pos.x)+", "+str(robot_pos.y)+", "+str(robot_pos.z))
