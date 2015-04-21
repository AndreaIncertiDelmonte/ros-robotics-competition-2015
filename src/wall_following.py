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

	scan = None
	wallDistance = 0.0
	maxSpeed = 0.0
	direction = 0.0
	kProp = 0.0
	kInt = 0.0
	kDiff = 0.0
	angleCoef = 0.0
	error = 0.0
	errorSum = 0.0
	errorDiff = 0.0
	distMin = 0	#minimum distance masured by sensor
	angleMin = 0   #angle, at which was measured the shortest distance
	go = 1	#in case of obstacle, change to 0

	def __init__(self, wallDistance, maxSpeed, direction, kProp, kInt, kDiff, angleCoef):
		"""
		Constructor
		Ex. WallFollowing(0.2, 0.1, 1, 5, 0, 5, 1)
		"""
		self.wallDistance = wallDistance
		self.maxSpeed = maxSpeed
		self.direction = direction
		self.kProp = kProp
		self.kInt = kInt
		self.kDiff = kDiff
		self.angleCoef = angleCoef
		self.error = 0.0
		self.errorSum = 0.0
		self.distMin = 0.0	#minimum distance measured by sensor
		self.angleMin = 0.0   #angle, at which was measured the shortest distance
		self.go = 1        #in case of obstacle, change to 0

	
	def computeLaserScanData(self):	
		"""
		Extract data from LaserScan
		"""

		#New model from right to left
		ileftEnd = 450
		icenter = 270
		irightBeg = 90

		size = len(self.scan.ranges)

		if self.direction == 1:
			#Left wall following
			minIndex = icenter
			maxIndex = ileftEnd
			iMinException = 450
			
		elif self.direction == -1:
			#Right wall following	
			minIndex = irightBeg
			maxIndex = icenter
			iMinException = 90

		print("MinIndex {0}".format(minIndex))
		print("MaxIndex {0}".format(maxIndex))
		
		self.distMin = min(self.scan.ranges[minIndex:maxIndex])
		print("\nDistMIn {0}".format(self.distMin))

		try:
			iMin = self.scan.ranges.index(self.distMin)
		except:
			iMin = iMinException

		print("iMin {0}".format(iMin))

		self.angleMin = self.scan.angle_min + (self.scan.angle_increment * iMin)

		self.distFront = self.getDistFront()
		
		print("DistFront {0}".format(self.distFront))

		self.errorDiff = 2 * (self.distMin - self.wallDistance) - self.error

		self.error = self.distMin - self.wallDistance
		print("Error {0}".format(self.error))
		self.errorSum = self.errorSum + self.error
		print("ErrorDiff {0}".format(self.errorDiff))
		print("ErrorSum {0}".format(self.errorSum))


	def getDistFront(self):
		"""
		Returns the min distance into the fron 30 degrees cone
		"""

		return min(self.scan.ranges[240:300])


	def computeVelocities(self):
		"""
		Applies a PID controller and computes angular and linear velocities
		"""

		if self.scan == None or len(self.scan.ranges) == 0:
			self.speedX = 0.0
			self.turnZ = 0.0
			return

		self.computeLaserScanData()

		#Speeds calculation

		#PID regulator		
		self.turnZ = self.direction * (self.kProp * self.error + self.kInt * self.errorSum + self.kDiff * self.errorDiff) \
			+ self.angleCoef * (self.angleMin - (numpy.pi * self.direction / 2));    

		if self.distFront < self.wallDistance:
			print("self.distFront < self.wallDistance")
			self.speedX = 0

		elif self.distFront < self.wallDistance * 2:
			print("self.distFront < self.wallDistance * 2")
			self.speedX = self.maxSpeed * 0.5

		elif abs(self.angleMin) > 1.75:
			print("abs(self.angleMin) > 1.75")
			self.speedX = 0.4 * self.maxSpeed
		else:
			print("Else")
			self.speedX = self.maxSpeed

		#Upper bound for turn Z
		if abs(self.turnZ) > 0.15:
			self.turnZ = numpy.sign(self.turnZ) * 0.15

		print("speedX {0}".format(self.speedX))
		print("turnZ {0}".format(self.turnZ))


	def getLinearVelocity(self):

		return self.speedX


	def getAngularVelocity(self):

		return self.turnZ


	def updateScanData(self, scan):

		self.scan = scan

	