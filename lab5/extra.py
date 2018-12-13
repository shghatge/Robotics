#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import pi
import numpy as np

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
    self.im_ind = 0

  def translate_robot(self,distance):

      move_cmd = Twist()
      move_cmd.linear.x = 0.2
      move_cmd.angular.z = 0.1
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
    global last, im_ind
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    dst = cv2.dilate(dst,None)
    image[dst>0.01*dst.max()] = [0,0,255]
    cv2.imwrite('./dumps/img_'+str(self.im_ind)+".jpg",image)
    # cv2.waitKey(1000)
    self.im_ind += 1
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20, 100, 100])
    upper_yellow = numpy.array([ 30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    maskMiddle = cv2.inRange(hsv, lower_yellow, upper_yellow)
    maskLeft = cv2.inRange(hsv, lower_yellow, upper_yellow)
    maskRight = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    maskMiddle[0:search_top, 0:w] = 0
    maskMiddle[search_bot:h, 0:w] = 0
    maskMiddle[search_top:search_bot, w/4:3*w/4] = 0
    maskLeft[0:search_top, 0:w] = 0
    maskLeft[search_bot:h, 0:w] = 0
    maskLeft[search_top:search_bot, 0:3*w/4] = 0
    maskRight[0:search_top, 0:w] = 0
    maskRight[search_bot:h, 0:w] = 0
    maskRight[search_top:search_bot, w/4:w] = 0
    Mm = cv2.moments(maskMiddle)
    if Mm['m00']>0:
      ret, thresh = cv2.threshold(maskMiddle,127,255,1)
      contours,h3 = cv2.findContours(thresh,1,2)
      ret1,thresh1 = cv2.threshold(maskLeft,127,255,1)
      contours1,h1 = cv2.findContours(thresh1,1,2)
      ret2,thresh2 = cv2.threshold(maskRight,127,255,1)
      contours2,h2 = cv2.findContours(thresh2,1,2)
      print(len(contours1)," ",len(contours2))
      if(len(contours2) > len(contours1)):
        #mask left
        print("masking left")
        mask[search_top:search_bot, 0:w/2] = 0
      elif(len(contours1) > len(contours2) ):
        #mask right
        print("masking right")
        mask[search_top:search_bot, w/2:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0 :   
      # M = cv2.moments(mask)        
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    else:
      mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
      mask2[0:search_top, 0:w] = 0
      mask2[search_bot:h, 0:w] = 0
      M = cv2.moments(mask2)
      # M = cv2.moments(mask)        
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