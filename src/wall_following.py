#!/usr/bin/env python
import rospy
import numpy
import math
from random import randint

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan


class WallFollowing(object):    

	_scan = None
	_wall_distance = 0.0
	_max_speed = 0.0
	_direction = 0.0
	_k_prop = 0.0
	_k_int = 0.0
	_k_diff = 0.0
	_angle_coef = 0.0


	def __init__(self, wall_distance, max_speed, direction, k_prop, k_int, k_diff, angle_coef):
		"""
		Constructor
		Ex. WallFollowing(0.2, 0.1, 1, 5, 0, 5, 1)
		"""
		self._wall_distance = wall_distance
		self._max_speed = max_speed
		self._direction = direction
		self._k_prop = k_prop
		self._k_int = k_int
		self._k_diff = k_diff
		self._angle_coef = angle_coef
		self._error = 0.0
		self._error_sum = 0.0
		self._dist_min = 0.0	#minimum distance measured by sensor
		self._angle_min = 0.0   #angle, at which was measured the shortest distance
		self._go = 1        #in case of obstacle, change to 0

	
	def _compute_laser_scan_data(self):		
		"""
		Extract data from LaserScan
		"""

		# From right to left
		i_left_end = 450
		i_center = 270
		i_right_beg = 90

		size = len(self._scan.ranges)

		if self._direction == 1:
			# Left wall following
			min_index = i_center
			max_index = i_left_end
			i_min_exception = 450
			
		elif self._direction == -1:
			# Right wall following	
			min_index = i_right_beg
			max_index = i_center
			i_min_exception = 90

		print("min_index {0}".format(min_index))
		print("max_index {0}".format(max_index))
		
		self._dist_min = min(self._scan.ranges[min_index:max_index])
		print("\nDist MIn {0}".format(self._dist_min))

		try:
			i_min = self._scan.ranges.index(self._dist_min)
		except:
			i_min = i_min_exception

		print("i_min {0}".format(i_min))

		self._angle_min = self._scan.angle_min + (self._scan.angle_increment * i_min)

		self._dist_front = self._get_dist_front()
		
		print("_dist_front {0}".format(self._dist_front))

		self._error_diff = 2*(self._dist_min - self._wall_distance) - self._error

		self._error = self._dist_min - self._wall_distance
		print("_error {0}".format(self._error))
		self._error_sum = self._error_sum + self._error
		print("_error_diff {0}".format(self._error_diff))
		print("_error_sum {0}".format(self._error_sum))


	def _get_dist_front(self):
		"""
		Returns the min distance into the fron 30 degrees cone
		"""

		return min(self._scan.ranges[240:300])


	def compute_velocities(self):
		"""
		Applies a PID controller and computes angular and linear velocities
		"""

		if self._scan == None or len(self._scan.ranges) == 0:
			self._speed_x = 0.0
			self._turn_z = 0.0
			return

		self._compute_laser_scan_data()

		# Speeds calculation

		# PID regulator		
		self._turn_z = self._direction*(self._k_prop*self._error + self._k_int*self._error_sum \
			+ self._k_diff*self._error_diff) + self._angle_coef*(self._angle_min \
			- (numpy.pi*self._direction / 2));    

		if self._dist_front < self._wall_distance:
			print("self._dist_front < self._wall_distance")
			self._speed_x = 0

		elif self._dist_front < self._wall_distance * 2:
			print("self._dist_front < self._wall_distance * 2")
			self._speed_x = self._max_speed*0.5

		elif abs(self._angle_min) > 1.75:
			print("abs(self._angle_min) > 1.75")
			self._speed_x = 0.4 * self._max_speed
		else:
			print("Else")
			self._speed_x = self._max_speed

		#Upper bound for turn Z
		if abs(self._turn_z) > 0.15:
			self._turn_z = numpy.sign(self._turn_z) * 0.15

		print("_speed_x {0}".format(self._speed_x))
		print("_turn_z {0}".format(self._turn_z))


	def get_linear_velocity(self):

		return self._speed_x


	def get_angular_velocity(self):

		return self._turn_z


	def update_scan_data(self, scan):

		self._scan = scan

