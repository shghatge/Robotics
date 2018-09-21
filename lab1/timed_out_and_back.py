#!/usr/bin/env python

""" timed_out_and_back.py - Version 1.2 2014-12-14

    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import thread
import os
import time
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):

        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        rate = 50
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0 meters per second 
        linear_speed = 0.0
        
        # Set the travel distance to 0.0 meters
        goal_distance = 0.0
        
        
        # Set the rotation speed to 0 radians per second
        angular_speed = 0.0
        
        # Set the rotation angle to 0 radians 
        goal_angle = 0
        
	T = 'T'
	Q = 'Q'
	R = 'R'

	opt = ''

        while opt != 'Q':
        
		opt = input("Input T for translation, R for rotation or Q for quit: ")

		if(opt == 'Q'):
			break
		
		# Initialize the movement command
		move_cmd = Twist()

		if(opt == 'T'):

			goal_distance = float(input("Input the distance to translate: "))
			goal_angle = 0

			# Set the linear speed
			if(goal_distance <0):
				linear_speed = -0.2
			else:
				linear_speed = 0.2
			move_cmd.linear.x = linear_speed
			linear_duration = goal_distance / linear_speed
			# Move for a time to go the desired distance
			ticks = int(linear_duration * rate)
			    
			for t in range(ticks):
				self.cmd_vel.publish(move_cmd)
				r.sleep()
		    
             		# Stop the robot 
            		move_cmd = Twist()
            		self.cmd_vel.publish(move_cmd)
            		rospy.sleep(1) 

		if(opt == 'R'):

			goal_angle = float(input("Input the angle to translate in radians: "))
			goal_distance = 0

			# Set the angular speed
			if(goal_angle <0):
				angular_speed = -2
			else:
				angular_speed = 2
			move_cmd.angular.z = angular_speed
			angular_duration = goal_angle / angular_speed
			# Move for a time to turn to the desired angle
			ticks = int(angular_duration * rate)
			    
			for t in range(ticks):
				self.cmd_vel.publish(move_cmd)
				r.sleep()
		    
             		# Stop the robot 
            		move_cmd = Twist()
            		self.cmd_vel.publish(move_cmd)
            		rospy.sleep(1) 
          
          
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        os.system('roslaunch rbx1_bringup fake_turtlebot.launch &')
	time.sleep(5)
        os.system('rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz &')
	time.sleep(5)
	OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

