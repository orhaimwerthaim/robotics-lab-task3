#!/usr/bin/env python
import pdb
import os, rospkg
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rospy import init_node, is_shutdown
from sensor_msgs.msg import LaserScan
import math
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from std_msgs.msg import String


class pub_angles_from_colored_obj:
    def set_colored_object_center(self):
        self.object_color = 'blue'


        cv_image2 = self.bridge.imgmsg_to_cv2(self.sensor_image)
        #cv_image2 = cv.resize(cv_image2, (self.image_width, self.image_width))
        # cv.imshow("Image original", cv_image2)
        color_bounds = color_bounds = self.get_colors_range()[self.object_color]
        hsv = cv.cvtColor(cv_image2, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_bounds[0], color_bounds[1])

        # remove random dots
        kernelOpen = np.ones((5, 5))
        kernalClose = np.ones((20, 20))

        maskOpen = cv.morphologyEx(mask, cv.MORPH_OPEN, kernelOpen)
        maskClose = cv.morphologyEx(maskOpen, cv.MORPH_CLOSE, kernelOpen)

        maskFinal = maskClose

        _, cnt, hierarchy = cv.findContours(maskFinal.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        if self.a:
            self.a = False
            #cv.imshow("reg", cv_image2)
            #cv.imshow("hasv", hsv)
            #cv.imshow("mask", mask)
            #cv.waitKey(0)
            #import pdb;
            #pdb.set_trace()
        self.color_object_center = self.null_value
        if not cnt is None and len(cnt) > 0:
            M = None
            maxSize = 0
            #if self.a:
                #self.a = False
                #import pdb;
                #pdb.set_trace()
            for x in cnt:
                p = cv.moments(x)
                rospy.loginfo('possible m00:%f', p["m00"])
                if p["m00"] > maxSize:
                    maxSize = p["m00"]
                    M = p

            if maxSize > 50:
            #M = cv.moments(cnt[0])
                cX = int(M["m10"] / M["m00"])
                rospy.loginfo('m00:%f', M["m00"])
                rospy.loginfo('len(cnt):%f', len(cnt))

                #import pdb;
                #pdb.set_trace()
                self.color_object_center = cX - (self.image_width / 2)
                rospy.loginfo('center of object Cx: %f, center:%f', cX, self.color_object_center)

        self.pub_angles_from_colored_object.publish(self.color_object_center)

        rospy.loginfo('center of object is: %f', self.color_object_center)
    def get_color_obj_dist(self):
        if self.color_object_center > -1:
            if self.face_obejct():
                return self.forward_dist
        else:
            return  None

    def callback_get_image(self, data):
        self.sensor_image = data
        self.set_colored_object_center()

    def callback_get_object_color(self,data):
        self.object_color = data.data

    def __init__(self):
        self.image_width = 800
        self.null_value = 900000
        self.a = True
        self.object_color = ''
        self.color_object_center = self.null_value
        self.bridge = CvBridge()
        self.pub_angles_from_colored_object = rospy.Publisher('/angles_from_colored_object', Float64, queue_size=10)
        self.sub_color_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_get_image)
        #/usb_cam/image_raw
        #/camera/image_raw
        #rospy.Subscriber('/object_color', String, self.callback_get_object_color)


    def get_colors_range(self):

        lower_blue = np.array([15, 000, 0])
        upper_blue = np.array([17, 200, 200])

        lower_red = np.array([80, 100, 100])
        upper_red = np.array([255, 255, 255])

        lower_yellow = np.array([20, 0, 0])
        upper_yellow = np.array([40, 255, 255])

        lower_green = np.array([40, 0, 0])
        upper_green = np.array([80, 255, 255])

        #lower_blue = np.array([80, 0, 0])
        #upper_blue = np.array([130, 255, 255])
        # define range of blue color in HSV
        lower_purple = np.array([130, 0, 0])
        upper_purple = np.array([255, 255, 255])

        res = {'blue': (lower_blue, upper_blue), 'green': (lower_green, upper_green),
               'red': (lower_red, upper_red), 'purple': (lower_purple, upper_purple), 'yellow': (lower_yellow, upper_yellow)}
        return res
if __name__ == '__main__':
    rospy.init_node('pub_angles_from_colored_obj', anonymous=True)
    pub_angles_from_colored_obj()
    rospy.spin()
