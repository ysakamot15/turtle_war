#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelStates 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from matplotlib import pyplot as plt


class AbstractBot(object):
    __metaclass__ = ABCMeta

    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name

        # bumper state
        self.bumper = BumperEvent()
        self.center_bumper = False
        self.left_bumper = False
        self.right_bumper = False
        
        self.red_score = 0
        self.blue_score = 0
        self.green_score =0 

        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # velocity publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

        # bumper subscrivre
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperCallback)

        # camera subscriver
        # please uncoment out if you use camera
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.imageCallback)

    # bumper topic call back sample
    # update bumper state
    def bumperCallback(self, data):
        if data.bumper == 0:
            if data.state == 1:
                self.left_bumper = True
            else:
                self.left_bumper = False
                
        if data.bumper == 1:
            if data.state == 1:
                self.center_bumper = True
            else:
                self.center_bumper = False

        if data.bumper == 2:
            if data.state == 1:
                self.right_bumper = True
            else:
                self.right_bumper = False

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # RGB表色系からHSV表色系に変換                                                    
            #hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            color = ('b','g','r')
            for i,col in enumerate(color):
                histr = cv2.calcHist([cv_image],[i],None,[256],[0,256])
                #print col, i , histr[180:256]
                if col  == 'r':
                    self.red_score = np.sum(histr[150:256])
                if col == 'g':
                    self.green_score = np.sum(histr[150:256])
                if col == 'b':
                    self.blue_score = np.sum(histr[150:256])
            print self.red_score, self.green_score, self.blue_score
            print type(histr)
            #plt.plot(histr,color = col)
            #plt.xlim([0,256])
            #plt.show()
            #color_min = np.array([200,128,0])
            #color_max = np.array([255,255,255])
            # マスク画像を生成                                                 
            #color_mask = cv2.inRange(hsv_image, color_min, color_max);
            # 画像配列のビット毎の倫理席。マスク画像だけが抽出される。            
            #cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)
            #print cv_image2
            #cv_image2 = cv2.cvtColor(cv_image2, cv2.COLOR_HSV2BGR)
            
            

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        #cv2.imshow("Image window2", hsv_image)
        cv2.waitKey(3)

    @abstractmethod
    def strategy(self):
        pass

