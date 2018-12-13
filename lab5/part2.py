#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import pi

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


  def image_callback(self, msg):
    global last
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20, 100, 100])
    upper_yellow = numpy.array([ 30, 255, 255])
    lower_green = numpy.array([50,100,100])
    upper_green = numpy.array([70,255,255])
    lower_blue = numpy.array([100,150,0])
    upper_blue = numpy.array([140,255,255])
    lower_red = numpy.array([0,50,50])
    upper_red = numpy.array([10,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask2 = cv2.inRange(hsv, lower_green, upper_green)
    mask3 = cv2.inRange(hsv, lower_blue, upper_blue)
    mask4 = cv2.inRange(hsv, lower_red, upper_red)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0
    mask3[0:search_top, 0:w] = 0
    mask3[search_bot:h, 0:w] = 0
    mask4[0:search_top, 0:w] = 0
    mask4[search_bot:h, 0:w] = 0
    M2 = cv2.moments(mask2)
    M3 = cv2.moments(mask3)
    M4 = cv2.moments(mask4)    
    turn = 0
    if M2['m00'] > 0:
      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])
      self.twist.linear.x = 0.1
      self.cmd_vel_pub.publish(self.twist)
      if(cy2>378):
        mask[search_top:search_bot, w/2:w] = 0
    elif M3['m00'] > 0:
      cx3 = int(M3['m10']/M3['m00'])
      cy3 = int(M3['m01']/M3['m00'])
      self.twist.linear.x = 0.1
      self.cmd_vel_pub.publish(self.twist)
      if(cy3>378):
        mask[search_top:search_bot, 0:w/2] = 0
    elif M4['m00'] > 0:
      cx4 = int(M4['m10']/M4['m00'])
      cy4 = int(M4['m01']/M4['m00'])
      self.twist.linear.x = 0.2
      self.cmd_vel_pub.publish(self.twist)
      if(cy4>378):
        self.translate_robot(0.4)
        
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
follower = Follower()
rospy.spin()
# END ALL