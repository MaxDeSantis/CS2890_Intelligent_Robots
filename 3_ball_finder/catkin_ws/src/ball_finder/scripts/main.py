#!/usr/bin/env python3
import roslib
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from ball_finder.msg import BallLocation
# from assn3.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError


class Detector:
    def __init__(self):
        self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
        self.locpub = rospy.Publisher('/ball_detector/ball_location', BallLocation, queue_size=1)

        self.bridge = CvBridge()
        self.bearing = -1
        self.distance = -1
        self.upper_bound = 70;
        self.lower_bound = 150;
        self.image_topic = rospy.get_param('~image', "/camera/rgb/image_raw")
        rospy.Subscriber(self.image_topic, Image, self.handle_image)
        #rospy.Subscriber('/scan', LaserScan, self.handle_scan)
        
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

        
        yellow = np.where( (im_hsv[:, :, 0] < 80) & (im_hsv[:, :, 1] >= 90) & (im_hsv[:, :, 2] >= 90))
        r, c = yellow
        kernel = np.ones((4, 4), np.float32) / 16 # introduces blur to remove noise, not sure if good idea

        image[r, c] = (0, 0, 0)
        image = cv2.filter2D(image, -1, kernel)
        sum = 0
        for i in c:
            sum = sum + i
        avg = sum / len(c)
        print(len(c), " ", avg)

        if len(c) >= 1000: #passes the test
            image[:, int(avg)] = (0, 255, 0)
            self.bearing = int(avg)
        else:
            self.bearing = -1

        self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image.
        # Feel free to change the values in the image variable
        # in order to see what is going on
        # Here we publish the modified image; it can be
        # examined by running image_view
        # def handle_scan(self, msg):
        # If the bearing is valid, store the corresponding range
        # in self.distance. Decide what to do if range is NaN.

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            location = BallLocation()
            location.bearing = self.bearing
            #location.distance = self.distance
            location.distance = 0
            self.locpub.publish(location)
            
            
rospy.init_node('ball_detector')
detector = Detector()
detector.start()