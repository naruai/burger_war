#!/usr/bin/env python
# -*- coding: utf-8 -*-

#######################################
# dd_BotControl.py
# write by Finn Fumihiro Hatano 
#######################################

# ROSpy
import rospy

# Twist
from geometry_msgs.msg import Twist
# JointState
from sensor_msgs.msg import JointState

# Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Odometry
from nav_msgs.msg import Odometry

# LaserScan
from sensor_msgs.msg import LaserScan

# Sleep
from time import sleep


class dd_walk():
	def __init__(self, bot_name):
		#######################################
		# Define the local value 
		# and initialization
		#######################################
		
		# bot name 
		self.name = bot_name

		# Walk state : Active or Not
		self.active = False
		
		# Order Num
		self.OrderNum = 0
		
		# Walk state 
		#  'go'
		#  'go left'
		#  'go right'
		#  'back'
		#  'back left'
		#  'back right'
		#  'turn left'
		#  'turn right'
		#  'turn back'
		#  'sleep'
		#  'wait order'
		self.state = 'wait order'

		# robot wheel rot 
		self.wheel_rot_aR = 0.0
		self.wheel_rot_aL = 0.0
		self.wheel_rot_baseR = 0.0
		self.wheel_rot_baseL = 0.0
		self.pose_x = 0.0
		self.pose_y = 0.0
		
		# speed [m/s]
		# self.speed = 0.25
		# self.speed = 0.25;	self.rotate = 3.8;	self.waitTime = 0.4
		self.speed = 0.50;	self.rotate = 3.8;	self.waitTime = 0.4
		
		# One Step Size
		self.OneStep = 4
		self.Steps = 4
		
		# Direction 
		self.direct = 1
		
		# publisher
		self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
		
		##############################
		# subscriber
		##############################
		# joint_states subscriber
		self.jointState_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)
		
		# camera subscribver
		# for convert image topic to opencv obj
		self.img = None
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
		
		## odom subscriber
		#self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
		
		## lidar scan subscriber
		#self.scan = LaserScan()
		#self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
		##############################

	# camera image call back sample
	# comvert image topic to opencv object and show
	def imageCallback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)
		
		cv2.imshow("DD-Image window", self.img)
		cv2.waitKey(1)

	# update odometry state
	def odomCallback(self, data):
		self.pose_x = data.pose.pose.position.x
		self.pose_y = data.pose.pose.position.y
		#rospy.loginfo("Odom pose_x-y: {:.3f}-{:.3f}".format(self.pose_x,self.pose_y))

	# update lidar scan state
	def lidarCallback(self, data):
		self.scan = data
		rospy.loginfo(self.scan)

	def jointstateCallback(self, data):
		self.wheel_rot_aR = data.position[0]
		self.wheel_rot_aL = data.position[1]
		
		if self.active == True and self.state != 'wait order':
			if self.state == 'go' \
				and self.wheel_rot_aR >= (self.wheel_rot_baseR + self.Steps) \
				and self.wheel_rot_aL >= (self.wheel_rot_baseL + self.Steps):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			# ar >= tr*2*3.14/4 = tr*1.57
			elif self.state == 'go left' \
				and self.wheel_rot_aR >= (self.wheel_rot_baseR + self.Steps * self.rotate):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'go right' \
				and self.wheel_rot_aL >= (self.wheel_rot_baseL + self.Steps * self.rotate):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'back' \
				and self.wheel_rot_aR <= (self.wheel_rot_baseR - self.Steps)\
				and self.wheel_rot_aL <= (self.wheel_rot_baseL - self.Steps):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			# ar <= tr*2*3.14/4 = tr*1.57
			elif self.state == 'back left' \
				and self.wheel_rot_aR <= (self.wheel_rot_baseR - self.Steps * self.rotate):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'back right' \
				and self.wheel_rot_aL <= (self.wheel_rot_baseL - self.Steps * self.rotate):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'turn left' \
				and self.wheel_rot_aR >= (self.wheel_rot_baseR + self.Steps * self.rotate / 4) \
				and self.wheel_rot_aL <= (self.wheel_rot_baseL - self.Steps * self.rotate / 4):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'turn right' \
				and self.wheel_rot_aR <= (self.wheel_rot_baseR - self.Steps * self.rotate / 4) \
				and self.wheel_rot_aL >= (self.wheel_rot_baseL + self.Steps * self.rotate / 4):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
			elif self.state == 'turn back' \
				and self.wheel_rot_aR <= (self.wheel_rot_baseR - self.Steps * self.rotate / 2) \
				and self.wheel_rot_aL >= (self.wheel_rot_baseL + self.Steps * self.rotate / 2):
				self.debugSout()
				rospy.loginfo("{:>10} : Finish".format(self.state))
				self.active = False
			
		self.update()

	def update(self):
		twist = Twist()
		
		if self.active == False:
			twist.linear.x = 0
			twist.angular.z = 0
		
		elif self.state == 'go':
			twist.linear.x = self.speed
			twist.angular.z = 0
			
		elif self.state == 'go left':
			twist.linear.x = self.speed
			twist.angular.z = self.direct
		elif self.state == 'go right':
			twist.linear.x = self.speed
			twist.angular.z = -1 * self.direct
			
		elif self.state == 'back':
			twist.linear.x = -1 * self.speed
			twist.angular.z = 0
			
		elif self.state == 'back left':
			twist.linear.x = -1 * self.speed
			twist.angular.z = -1 * self.direct
			
		elif self.state == 'back right':
			twist.linear.x = -1 * self.speed
			twist.angular.z = self.direct
			
		elif self.state == 'turn left':
			twist.linear.x = 0
			twist.angular.z = self.direct
			
		elif self.state == 'turn right':
			twist.linear.x = 0
			twist.angular.z = -1 * self.direct
			
		elif self.state == 'turn back':
			twist.linear.x = 0
			twist.angular.z = -1 * self.direct
			
		else:
			twist.linear.x = 0
			twist.angular.z = 0
		
		#########################
		# Init
		#########################
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		# twist.angular.z = self.direct
		
		#########################
		# publish twist topic
		#########################
		# rospy.loginfo("Speed : {}".format(twist.linear.x))
		self.vel_pub.publish(twist)
		#########################
		
		if self.state != 'wait order' and self.active == False:
			self.state = 'sleep'
			rospy.loginfo("{:>10} : Start".format(self.state))
			sleep(self.waitTime)
			rospy.loginfo("{:>10} : Finish".format(self.state))
			self.state = 'wait order'
			rospy.loginfo("Watinig next order")

	def go(self, RepeatTimes=1):
		self.state = 'go'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def go_left(self, RepeatTimes=1):
		self.state = 'go left'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def go_right(self, RepeatTimes=1):
		self.state = 'go right'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def back(self, RepeatTimes=1):
		self.state = 'back'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def back_left(self, RepeatTimes=1):
		self.state = 'back left'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def back_right(self, RepeatTimes=1):
		self.state = 'back right'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()


	def turn_left(self, RepeatTimes=1):
		self.state = 'turn left'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def turn_right(self, RepeatTimes=1):
		self.state = 'turn right'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	def turn_back(self, RepeatTimes=1):
		self.state = 'turn back'
		rospy.loginfo("{:>10} : Start".format(self.state))
		self.OrderNum = self.OrderNum + 1
		self.wheel_rot_baseR = self.wheel_rot_aR
		self.wheel_rot_baseL = self.wheel_rot_aL
		self.Steps = self.OneStep * RepeatTimes
		self.active = True
		self.update()

	# debug out
	def debugSout(self):
		rospy.loginfo("Walking {:02d}:{:>10}, aR-L:{:.3f}-{:.3f} tR-L:{:.3f}-{:.3f} ".format( \
			bot.OrderNum, \
			bot.state, \
			bot.wheel_rot_aR, \
			bot.wheel_rot_aL, \
			bot.wheel_rot_baseR, \
			bot.wheel_rot_baseL))

# Sample Strategy
def Sample_Strategy01():
	# update work
	bot.go(5)
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()
	
	bot.turn_left()
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()
	
	bot.back()
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()
	
	bot.go_left()
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()
	
	bot.back_left()
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()

	bot.back_right(4)
	# while bot.active:
	while bot.state != 'wait order':
		bot.debugSout()
		bot.update()
		r.sleep()

# Sample Strategy
class Sample_Strategy02():
	def __init__(self, bot_name, iRate):
		#######################################
		# Define the local value 
		# and initialization
		#######################################
		# bot name 
		self.name = bot_name
		self.r = iRate

	def waitingOneProcess(self):
		while bot.state != 'wait order':
			bot.debugSout()
			bot.update()
			self.r.sleep()

	def strategy(self):
		bot.go(6);				self.waitingOneProcess()
		bot.back();				self.waitingOneProcess()
		bot.back_right();		self.waitingOneProcess()
		bot.back(2);			self.waitingOneProcess()
		bot.turn_right();		self.waitingOneProcess()
		bot.turn_left();		self.waitingOneProcess()
		bot.go(8);				self.waitingOneProcess()
		bot.turn_right();		self.waitingOneProcess()
		bot.turn_left(0.2);		self.waitingOneProcess()
		bot.back(6);			self.waitingOneProcess()
		bot.go(10);				self.waitingOneProcess()
		bot.go_right();			self.waitingOneProcess()
		bot.go(3);				self.waitingOneProcess()
		bot.turn_left(0.3);		self.waitingOneProcess()
		bot.turn_right(2);		self.waitingOneProcess()
		bot.turn_left(2);		self.waitingOneProcess()
		
		while not rospy.is_shutdown():
			self.waitingOneProcess()


if __name__ == '__main__':
	rospy.init_node('dd test')
	bot = dd_walk('dd_walk')
	
	r = rospy.Rate(5) # change speed 1fps
	
	# Sample_Strategy01()
	st = Sample_Strategy02('dd test - S02', r)
	st.strategy()


