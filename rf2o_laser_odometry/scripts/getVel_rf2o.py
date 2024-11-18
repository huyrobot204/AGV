#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sti_msgs.msg import *
import math
import time
import threading
import signal
import os

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class getVel_pose():
	def __init__(self):
		print("ROS Initial: getVel_pose!")
		rospy.init_node('getVel_pose', anonymous = False) # False
		self.rate = rospy.Rate(1)
		# -- 
		# ------------------------ LMS100 ------------------------------------ #
		rospy.Subscriber('/odom_lms100', Odometry, self.callback_odomFf2o_lms100)
		self.odom_lms100 = Odometry()
		self.is_readOdom_lms100 = 0
		self.timeStampe_odomFf2o = rospy.Time.now()
        # -
		self.poseLms100_now = Pose()
		self.saveTime_lms100_now = time.time()
        # -
		self.poseLms100_bef = Pose()
		self.saveTime_lms100_bef = time.time()

		# -- 
		self.pub_odomLms100 = rospy.Publisher("/lms100_odomRf2o", Odometry, queue_size = 60)
		self.lms100_odomRf2o = Odometry()

		# --------------------------- TIM551 --------------------------------- #
		rospy.Subscriber('/odom_tim551', Odometry, self.callback_odomFf2o_tim551)
		self.odom_tim551 = Odometry()
		self.is_readOdom_tim551 = 0
		self.timeStampe_odomFf2o_tim551 = rospy.Time.now()
        # -
		self.poseTim551_now = Pose()
		self.saveTime_tim551_now = time.time()
        # -
		self.poseTim551_bef = Pose()
		self.saveTime_tim551_bef = time.time()
		# ---- #
		self.pub_odomTim551 = rospy.Publisher("/tim551_odomRf2o", Odometry, queue_size = 60)
		self.tim551_odomRf2o = Odometry()
		# ------------------------------------------------------------ #

	def callback_odomFf2o_lms100(self, data):
		self.odom_lms100 = data
		self.is_readOdom_lms100 = 1
		self.timeStampe_odomFf2o = rospy.Time.now()
		# -
		self.lms100_odomRf2o = self.odom_lms100
		self.lms100_odomRf2o.header.frame_id = "odom_lms100"
		# --
		self.poseLms100_now = self.odom_lms100.pose.pose
		self.saveTime_lms100_now = time.time()
		# --
		new_robot, new_goal = self.fakePose_robotFollowGoal(self.poseLms100_bef, self.poseLms100_now)
		# --
		delta_x = 0.0
		delta_y = 0.0
		vel_y = 0.0
		vel_x = 0.0
		# -
		delta_x = new_robot.position.x
		delta_y = new_robot.position.y
		# --
		delta_time = 0.0
		delta_time = (self.saveTime_lms100_now - self.saveTime_lms100_bef)%60 
		# --
		if delta_time < 0.5:
			vel_y = delta_y/delta_time
			vel_x = delta_x/delta_time
		# --
		self.saveTime_lms100_bef = self.saveTime_lms100_now

		# print ("------------")
		# print ("delta_y: ", delta_y)
		# print ("delta_time: ", delta_time)

		self.poseLms100_bef.position = self.poseLms100_now.position
		self.poseLms100_bef.orientation = self.poseLms100_now.orientation
		# --
		self.lms100_odomRf2o.twist.twist.linear.x = vel_x*-1
		self.lms100_odomRf2o.twist.twist.linear.y = vel_y*-1
		self.pub_odomLms100.publish(self.lms100_odomRf2o)
		# print ("-- Y: " + str(delta_y) + " | T: " + str(delta_time))

	def callback_odomFf2o_tim551(self, data):
		self.odom_tim551 = data
		self.is_readOdom_tim551 = 1
		self.timeStampe_odomFf2o_tim551 = rospy.Time.now()
		# --
		self.tim551_odomRf2o.header.frame_id = "odom_tim551"
		self.tim551_odomRf2o.header.stamp = rospy.Time.now()
		self.tim551_odomRf2o.child_frame_id = self.odom_tim551.child_frame_id
		# --
		self.tim551_odomRf2o.pose = self.odom_tim551.pose
		self.tim551_odomRf2o.twist.twist.linear.x	 = self.odom_tim551.twist.twist.linear.x*-1
		self.tim551_odomRf2o.twist.twist.angular.z = self.odom_tim551.twist.twist.angular.z
		# --
		self.poseTim551_now = self.odom_tim551.pose.pose
		self.saveTime_tim551_now = time.time()
		new_robot, new_goal = self.fakePose_robotFollowGoal(self.poseTim551_bef, self.poseTim551_now)
		# --
		delta_x = 0.0
		delta_y = 0.0
		vel_x = 0.0
		vel_y = 0.0
		# -
		delta_x = new_robot.position.x
		delta_y = new_robot.position.y

		# --
		delta_time = 0.0
		delta_time = (self.saveTime_tim551_now - self.saveTime_tim551_bef)%60 
		if delta_time < 0.5:
			vel_y = delta_y/delta_time
			vel_x = delta_x/delta_time

		# print ("------------")
		# print ("delta_y: ", delta_y)
		# print ("delta_time: ", delta_time)

		self.poseTim551_bef.position = self.poseTim551_now.position
		self.poseTim551_bef.orientation = self.poseTim551_now.orientation
		self.saveTime_tim551_bef = self.saveTime_tim551_now
		# -
		self.tim551_odomRf2o.twist.twist.linear.x = vel_x*-1
		self.tim551_odomRf2o.twist.twist.linear.y = vel_y*-1
		# --
		self.pub_odomTim551.publish(self.tim551_odomRf2o)

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def angleLine_AB(self, pointA, pointB): # -- Angle Line Point A to Point B. | Point()
		d_x = pointB.x - pointA.x
		d_y = pointB.y - pointA.y
		ang = 0
		if d_x == 0:
			if d_y >= 0:
				ang = pi/2.
			else:
				ang = -pi/2.
		else:
			if d_y == 0:
				if d_x > 0:
					ang = 0
				else:
					ang = pi
			else:
				ang = atan2(d_y, d_x)
				if ang > pi:
					ang = ang - 2*pi
				if ang < -pi:
					ang = 2*pi + ang
		return ang

	def fakePose_robotFollowGoal(self, robotPose, goalPose): # -- goal -> main.
		distancePoint = self.calculate_distance(robotPose.position, goalPose.position)
		# --
		angle_goalToRobot = self.angleLine_AB(goalPose.position, robotPose.position)
		# angle_robotToGoal = self.angleLine_AB(robotPose.position, goalPose.position)
		# --
		angleGoal = self.quaternion_to_euler(goalPose.orientation)
		angleRobot = self.quaternion_to_euler(robotPose.orientation)
		# -- 
		# print ("angleRobot: ", degrees(angleRobot))
		# print ("angleGoal: ", degrees(angleGoal))
		# print ("angle_goalToRobot: ", degrees(angle_goalToRobot))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		# -- 
		angleNew_goalToRobot = angle_goalToRobot - angleGoal
		angleNew_goalToRobot = self.limitAngle(angleNew_goalToRobot)
		# -- 
		# print ("angleNew_goalToRobot: ", degrees(angleNew_goalToRobot) )
		# -- 
		angleRobot_new = angleRobot - angleGoal
		angleRobot_new = self.limitAngle(angleRobot_new)
		# -- 
		poseRobot_new = Pose()
		poseRobot_new.position.x = cos(angleNew_goalToRobot)*distancePoint
		poseRobot_new.position.y = sin(angleNew_goalToRobot)*distancePoint
		poseRobot_new.orientation = self.euler_to_quaternion(angleRobot_new)

		# print ("poseRobot_new X: ", poseRobot_new.position.x)
		# print ("poseRobot_new Y: ", poseRobot_new.position.y)
		# print ("poseRobot_new R: ", degrees(angleRobot_new) )
		# print (str(round(poseGoal_new.position.x, 3)) + " | " + str(round(poseGoal_new.position.y, 3)))

		poseGoal_new = Pose()
		poseGoal_new.orientation = self.euler_to_quaternion(0)

		return poseRobot_new, poseGoal_new


	def run(self):
		while not rospy.is_shutdown():

			self.rate.sleep()
			
		# self.pub_cmdVel.publish(Twist())

def main():
	print('Starting main program')
	program = getVel_pose()
	program.run()
	print('Exiting main program')
	program.stop_run()

if __name__ == '__main__':
    main()