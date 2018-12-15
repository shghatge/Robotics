#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import pi
import tf
import math
from geometry_msgs.msg import Point,  Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class Follower:
  def __init__(self):
    self.rate = rospy.Rate(50)
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback, queue_size=1)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=500)
    self.twist = Twist()

  def translate_robot(self,distance):

      move_cmd = Twist()
      move_cmd.linear.x = 0.2
      linear_duration = distance / 0.2
      # Move for a time to go the desired distance
      ticks = int(linear_duration * 50)

      for t in range(ticks):
          self.cmd_vel_pub.publish(move_cmd)
          self.rate.sleep()

      move_cmd = Twist()
      self.cmd_vel_pub.publish(move_cmd)
      rospy.signal_shutdown("Reached Red")

  def rotate_robot(self, angle, direction):

      #angle_turned += ((direction) * angle)
      print("Rotating by angle "+str(angle))
      move_cmd = Twist()
      move_cmd.angular.z = direction * 1
      angular_duration = angle / 1
      # Move for a time to turn to the desired angle
      ticks = int(angular_duration * 50)

      for t in range(ticks):
          cmd_vel.publish(move_cmd)
          self.rate.sleep()

      # Stop the robot 
      move_cmd = Twist()
      cmd_vel.publish(move_cmd)

  def getDist(self, pt1, pt2):

    distance = math.sqrt( ( pt1[0] - pt2[0] ) ** 2 + ( pt1[1] - pt2[1] ) ** 2) 
    return distance

  def getAngle(self, pt1, pt2):

    if( ( pt1[0] - pt2[0] ) == 0):
      angle = math.pi / 2
    else:
      angle = math.atan( ( pt1[1] - pt2[1] ) / ( pt1[0] - pt2[0] ) );

    return angle

  def get_odom_pos(self):
    try:
        trans, rot = tf_listener.lookupTransform( odom_frame, base_frame, rospy.Time(0) )
    except  (tf.Exception,  tf.ConnectivityException,  tf.LookupException):
        rospy.loginfo( "TF Exception" )
        return

    print("Positions "+str(trans[0])+" "+str(trans[1]))
    return Point(trans[0], trans[1], trans[2])

  def get_odom_pose(self):
    try:
        trans, rot = tf_listener.lookupTransform( odom_frame, base_frame, rospy.Time(0) )
    except  (tf.Exception,  tf.ConnectivityException,  tf.LookupException):
        rospy.loginfo( "TF Exception" )
        return
    print("pose")
    return quat_to_angle(Quaternion(*rot))

  def final_goto_goal(self):

    self.translate_robot(0)
    robot_pose = self.get_odom_pose()
    robot_pos = self.get_odom_pos()
    angle = self.getAngle( (robot_pos.x, robot_pos.y), (-2.362, -1.1715) ) - robot_pose
    sign = 1
    if(angle<0): 
      sign = -1
    self.rotate_robot(abs(angle), sign)
    # rospy.sleep(0.1)
    dist = self.getDist( (robot_pos.x, robot_pos.y), (-2.362, -1.1715))
    # total_dist += dist
    self.translate_robot(dist)
    self.translate_robot(0)

  def image_callback(self, msg):
    global last
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20, 100, 100])
    upper_yellow = numpy.array([ 30, 255, 255])
    lower_red = numpy.array([0,50,50])
    upper_red = numpy.array([10,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask4 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    maskLeft = cv2.inRange(hsv, lower_yellow, upper_yellow)
    maskRight = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask4[0:search_top, 0:w] = 0
    mask4[search_bot:h, 0:w] = 0
    maskLeft[0:search_top, 0:w] = 0
    maskLeft[search_bot:h, 0:w] = 0
    maskLeft[search_top:search_bot, 0:3*w/4] = 0
    maskRight[0:search_top, 0:w] = 0
    maskRight[search_bot:h, 0:w] = 0
    maskRight[search_top:search_bot, w/4:w] = 0
    M4 = cv2.moments(mask4)    
    if M4['m00'] > 0:
      ret1,thresh1 = cv2.threshold(maskLeft,127,255,1)
      contours1,h1 = cv2.findContours(thresh1,1,2)
      ret2,thresh2 = cv2.threshold(maskRight,127,255,1)
      contours2,h2 = cv2.findContours(thresh2,1,2)
      ret, thresh = cv2.threshold(mask4,127,255,1)
      contours,h = cv2.findContours(thresh1,1,2)
      for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        if len(approx)>14:
          self.final_goto_goal()
      if(len(contours2) > len(contours1)):
        #mask left
        mask[search_top:search_bot, 0:w/2] = 0
      if(len(contours1) > len(contours2)):
        #mask right
        mask[search_top:search_bot, w/2:w] = 0  
    M = cv2.moments(mask)
    if M['m00'] > 0 :      
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower',disable_signals=True)
tf_listener  =  tf.TransformListener()
odom_frame  =  '/odom'
robot_pose = 0.0

#Subscribe to scan and odomotry and publish cmd_vel

cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
rate = rospy.Rate(50)

try:
    tf_listener.waitForTransform( odom_frame,  '/base_footprint' ,  rospy.Time(),  rospy.Duration( 1.0 ))
    base_frame  =  '/base_footprint'
except(tf.Exception,  tf.ConnectivityException,  tf.LookupException):
    rospy.loginfo( "Cannot find transform between /odom and /base_link or /base_footprint" )
    rospy.signal_shutdown( "tf Exception" )
follower = Follower()
rospy.spin()
# END ALL
