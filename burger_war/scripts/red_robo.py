#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.

by Takuya Yamaguchi @dashimaki360
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np

def dist2d(p1,p2):
    dx=p1[0]-p2[0]
    dy=p1[1]-p2[1]
    return math.sqrt(dx*dx+dy*dy)

def countblue(img,p0,p1):
    cnt=0
    for idy in range (p1[1]-p0[1]):
        iy=p0[1]+idy
        for idx in range (p1[0]-p0[0]):
            ix=p0[0]+idx
            col=img[iy,ix,:]
            b,g,r=col
            if b/2>(g/2+r/2):
                cnt+=1
                col=255,64,64
            img[iy,ix,:]=col
    return cnt

class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # init
        self.mypos=[0.0, 0.0]
        self.state=0 # initial
        self.angleadj=0.0 # initial
        self.frontdist=1.1 # tentative [m]
        self.reardist=0.3 # tentative [m]
        self.pose_x=0.0 # dummy
        self.pose_y=0.0 # dummy
        self.ranges=np.zeros(360)
        self.blue=np.zeros((6),np.int)
        rospy.loginfo("#### init")

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # strategy
            rospy.loginfo("#strategy state {}".format(self.state))
            # angle
            rospy.loginfo("#angle {}".format(self.blue))
            if (self.blue[0]>100) and (self.blue[1]>100):
                rospy.loginfo("#fine")
                if (self.blue[4]<10) or (self.blue[5]<10): # fine good
                    self.angleadj=0.0
                    rospy.loginfo("#little {}".format(self.angleadj))
                else:
                    if (self.blue[0]>self.blue[1]): # go right little
                        self.angleadj= 0.005
                    else:
                        self.angleadj=-0.005
                    rospy.loginfo("#little {}".format(self.angleadj))
            else:
                if (self.blue[0]>100):   # turn left
                    self.angleadj= 0.01
                elif (self.blue[1]>100): # turn right
                    self.angleadj=-0.01
                elif (self.blue[2]<1): # nothing
                    self.angleadj= 0.05
                else:
                    self.angleadj= 0.02
                rospy.loginfo("#little {}".format(self.angleadj))

            # update twist
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.angleadj
            frontdist=self.ranges[0]
            reardist =self.ranges[180]
            if ( (frontdist>0.2) and (frontdist<3.4) ):
                self.frontdist=frontdist
            if ( (reardist>0.2) and (reardist<3.4) ):
                self.reardist=reardist

            if self.state==4: # rotate left
                twist.linear.x=0.08
                if (abs(self.angleadj)<0.02):
                    self.state=10

            if self.state==3: # rotate left
                twist.linear.x=0.06
                if (abs(self.angleadj)<0.02):
                    self.state=4

            if self.state==2: # rotate left
                twist.linear.x=0.04
                if (abs(self.angleadj)<0.02):
                    self.state=3

            if self.state==1: # rotate left
                twist.linear.x=0.02
                if (abs(self.angleadj)<0.02):
                    self.state=2

            if self.state==0: # rotate left
                if (abs(self.angleadj)<0.008):
                    self.state=1


            if self.state==10: # go fwd fast
                if (abs(self.angleadj)>0.04):
                    self.state=12
                if (self.angleadj==0):
                    twist.linear.x=0.1
                else:
                    twist.linear.x=0.05
                if self.frontdist<=0.5:
                    twist.linear.x=0.05
                    self.state=11

            if self.state==11: # go fwd
                if (abs(self.angleadj)>0.04):
                    self.state=12
                twist.linear.x=0.05
                if self.frontdist<0.4:
                    twist.linear.x=-0.05
                    self.state=12

            if self.state==12: # go back fast
                twist.linear.x=-0.1
                if self.reardist<0.6:
                    twist.linear.x=-0.05
                    self.state=13

            if self.state==13: # go back
                twist.linear.x=-0.05
                if self.reardist<0.5:
                    self.state=20

            if self.state==21: # rotate for next
                if (abs(self.angleadj)<0.015):
                    self.state=0

            if self.state==20: # rotate for ignore current
                if (abs(self.angleadj)<0.015):
                    twist.angular.z = 0.02
                else:
                    self.state=21
                rospy.loginfo("STATE20: {}".format(abs(self.angleadj)))

            # publish twist topic
            rospy.loginfo("twist: {}".format(twist))
            self.vel_pub.publish(twist)
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)
        self.ranges=self.scan.ranges[:]

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #shape=self.img.shape
        cv2.putText(self.img,str(self.state),(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.angleadj),(310,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.pose_x),(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.pose_y),(10,70),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.frontdist),(10,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.reardist), (10,120),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.ranges[0]),(310,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.ranges[180]), (310,120),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.rectangle(self.img,(240,220),(320,260),(0,255,0),1)
        self.blue[0]=countblue(self.img,(240,220),(320,260))
        cv2.rectangle(self.img,(320,220),(400,260),(0,255,0),1)
        self.blue[1]=countblue(self.img,(320,220),(400,260))

        cv2.rectangle(self.img,(0,230),(240,250),(0,255,0),1)
        self.blue[2]=countblue(self.img,(0,230),(240,250))
        cv2.rectangle(self.img,(400,230),(639,250),(0,255,0),1)
        self.blue[3]=countblue(self.img,(400,230),(639,250))

        cv2.rectangle(self.img,(315,235),(326,246),(0,255,0),1)
        self.blue[4]=countblue(self.img,(315,235),(322,246))
        self.blue[5]=countblue(self.img,(319,235),(326,246))
        #rospy.loginfo("##blue: {}".format(self.blue))

        #cv2.putText(self.img,str(shape), (10,150),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.putText(self.img,str(self.blue), (10,150),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0))
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

        self.limg=np.zeros((400,400), np.float)
        cv2.line(self.limg,(200,0),(200,400),0.5,1)
        cv2.line(self.limg,(0,200),(400,200),0.5,1)
        ranges=self.ranges
        r=ranges[0]*200.0/2.0
        if r>340.0:
            r=340.0
        p0=[ 200.0, 200-r*200/2.0 ]
        for it in range(360):
            t=-math.pi*(it+90)/180.0
            r=ranges[it]*200.0/2.0
            if r>340.0:
                r=340.0
            p1=[ 200+r*math.cos(t), 200+r*math.sin(t) ]
            if dist2d(p0,p1)<20:
                cv2.line(self.limg,(int(p0[0]),int(p0[1])),(int(p1[0]),int(p1[1])),1.0,2)
            else:
                cv2.line(self.limg,(int(p0[0]),int(p0[1])),(int(p1[0]),int(p1[1])),0.2,2)
            p0=p1
        cv2.imshow("Lidar window", self.limg)
        cv2.waitKey(1)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        #rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        newpos=[ self.pose_x, self.pose_y ]
        dist=dist2d(newpos, self.mypos)
        if dist>0.01:
            self.mypos=newpos
            #rospy.loginfo("odom pose: {}".format(newpos, dist))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        #rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        #rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = AllSensorBot(use_lidar=True, use_camera=True, use_imu=True,
                       use_odom=True, use_joint_states=True)
    bot.strategy()


