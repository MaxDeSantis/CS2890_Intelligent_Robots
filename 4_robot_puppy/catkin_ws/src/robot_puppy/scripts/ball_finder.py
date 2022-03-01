#!/usr/bin/env python3
import roslib
import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import Image, LaserScan
from robot_puppy.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError


class Detector:
    def __init__(self):
        self.impub = rospy.Publisher('/puppy/image', Image, queue_size=1)
        self.locpub = rospy.Publisher('/puppy/ball_location', BallLocation, queue_size=1)

        self.bridge = CvBridge()
        self.bearing = -1
        self.distance = -1
        self.upper_bound = 70
        self.lower_bound = 170
        self.kernel_size = 8
        
        self.image_topic = rospy.get_param('~image', "/camera/rgb/image_raw")
        rospy.Subscriber(self.image_topic, Image, self.handle_image)
        rospy.Subscriber('/scan', LaserScan, self.handle_scan)
        
    def handle_image(self, msg):
        
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            (rows, columns, channels) = image.shape
            lower = int(rows/2 - self.upper_bound)
            upper = int(rows/2 + self.lower_bound)
            image = image[lower:upper, :]

            im_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)# Not sure if this is going to work as intended.

        kernel = np.ones((self.kernel_size, self.kernel_size), np.float32) / (self.kernel_size * self.kernel_size) # introduces blur to remove noise, not sure if good idea
        im_hsv = cv2.filter2D(im_hsv, -1, kernel)
        
        yellow = np.where( (im_hsv[:, :, 0] <= 70) & (im_hsv[:, :, 0] >= 0) & (im_hsv[:, :, 1] >= 130) & (im_hsv[:, :, 2] >= 130))
        r, c = yellow
        
        image[r, c] = (255, 0, 0)
        
        

        if len(c) >= 900: #passes the test
            sum = 0
            for i in c:
                sum = sum + i
            avg = sum / len(c)
            image[:, int(avg)] = (0, 255, 0)
            self.bearing = int(avg)
        else:
            self.bearing = -1

        self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image.

    def handle_scan(self, msg):
        if self.bearing >= 0:
            dist = msg.ranges[-self.bearing]
            if not math.isnan(dist):
                self.distance = dist
            else:
                self.distance = -1
        else:
            self.distance = -1
        #If the bearing is valid, store the corresponding range
        #in self.distance. Decide what to do if range is NaN.

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            location = BallLocation()
            location.bearing = self.bearing
            location.distance = self.distance
            self.locpub.publish(location)
         

rospy.init_node('ball_finder')
detector = Detector()
detector.start()
