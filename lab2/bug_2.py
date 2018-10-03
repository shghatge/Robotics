import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Callback function copies the distance of obstacle right in front and at the right most angle of view
def scan_callback(msg):
    global front_range, right_range 
    front_range = msg.ranges[ len( msg.ranges ) / 2 ]
    right_range = msg.ranges[-1]

#callback function to get the current position and pose of the robot
def odom_callback(odom_data):
    global robot_pos, robot_pose 
    robot_pos = odom_data.pose.pose.position
    robot_pose = odom_data.pose.pose.orientation

# These variables are updated as the robot moves, initialised to arbitrary values
front_range = 1.0
right_range = 0.8

# These variables store the position and pose of robot
robot_pos = [0, 0, 0]
robot_pose = [0, 0, 0, 1]

#Subscribe to scan and odomotry and publish cmd_vel

scan_top = rospy.Subscriber('scan', LaserScan, scan_callback)
odo_top = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

rospy.init_node(bug_2, anonymous = False)

