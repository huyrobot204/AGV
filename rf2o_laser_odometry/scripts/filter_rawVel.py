#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: BeeBee
# Date: 09/02/2023

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import *
import math
import time
import threading
import signal
import os

from math import atan2, sin, cos, sqrt, degrees, radians

class Filter_rawVel():
	def __init__(self):
		
		rospy.init_node('Filter_rawVel', anonymous=True)
		self.rate = rospy.Rate(20)

		rospy.Subscriber("/odom_lms100", Odometry, self.callback_odomRf2o)
		self.lms100_odomRf2o = Odometry()
		self.linear_x = 0.0

		# - Get param 
		self.kalman_rawVel_r = rospy.get_param('~kalman_rawVel_r', 0.1)
		self.kalman_rawVel_p = rospy.get_param('~kalman_rawVel_p', 1.0)
		self.kalman_rawVel_q = rospy.get_param('~kalman_rawVel_q', 0.1)

		self.pub_rawVel_filter = rospy.Publisher('/rawVel_filter', Twist, queue_size = 20)
		self.rawVel_filter = Twist()

		# - Kalman
		self.k_rawVel = self.x_now_rawVel = self.x_pre_rawVel = 0.0
		self.f1 = [self.kalman_rawVel_r, self.kalman_rawVel_p, self.kalman_rawVel_q] # r,p,q

		self.distance_runed = 0.0
		self.saveTime_odom = rospy.Time.now()
		
	def callback_odomRf2o(self, data):
		self.lms100_odomRf2o = data
		self.linear_x = self.lms100_odomRf2o.twist.twist.linear.x
		# -
		self.rawVel_filter.linear.x = self.kalman_rawVel(self.linear_x)
		raw = round(self.linear_x, 3)
		fil = round(self.rawVel_filter.linear.x, 3)
		print ("--------------")
		#print ("raw: " + str(raw) + " | Filter: " + str(fil) )
		# self.pub_rawVel_filter.publish(self.rawVel_filter)

		delta_time = (self.saveTime_odom - rospy.Time.now()).to_sec()
		delta_dis = delta_time*self.rawVel_filter.linear.x
		self.distance_runed += delta_dis

		self.saveTime_odom = rospy.Time.now()
		#print ("delta_dis: ", delta_dis)
		print ("distance_runed: ", round(self.distance_runed, 3))

	# - 
	def kalman_rawVel(self, input):
		self.k_rawVel = self.f1[1]/(self.f1[1] + self.f1[0])
		self.x_now_rawVel = self.x_pre_rawVel + self.k_rawVel*(input-self.x_pre_rawVel)
		self.f1[1] = (1-self.k_rawVel)*self.f1[1]+ math.fabs(self.x_pre_rawVel-self.x_now_rawVel)*self.f1[2]
		self.x_pre_rawVel = self.x_now_rawVel    
		return self.x_now_rawVel

	def run(self):
		while not rospy.is_shutdown():
			# self.move()
			# self.rawVel_filter.linear.x = self.my_kalman1(self.linear_x)
			# print ()

			# self.pub_rawVel_filter.publish(self.rawVel_filter)
			self.rate.sleep()


def main():
	print ("Start Program")
	program = Filter_rawVel()
	program.run()
 
if __name__ == '__main__':
	main()
