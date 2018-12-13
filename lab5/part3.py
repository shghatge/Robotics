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


  def image_callback(self, msg):
    global last
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20, 100, 100])
    upper_yellow = numpy.array([ 30, 255, 255])
    lower_red = numpy.array([0,50,50])
    upper_red = numpy.array([10,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask4 = cv2.inRange(hsv, lower_red, upper_red)
    maskLeft = cv2.inRange(hsv, lower_red, upper_red)
    maskRight = cv2.inRange(hsv, lower_red, upper_red)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask4[0:search_top, 0:w] = 0
    mask4[search_bot:h, 0:w] = 0
    maskLeft[0:search_top, 0:w] = 0
    maskLeft[search_bot:h, 0:w] = 0
    maskLeft[search_top:search_bot, 0:w/2] = 0
    maskRight[0:search_top, 0:w] = 0
    maskRight[search_bot:h, 0:w] = 0
    maskRight[search_top:search_bot, w/2:w] = 0
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
          self.translate_robot(1.36)
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
follower = Follower()
rospy.spin()
# END ALL