#!/usr/bin/env python3
import roslib
import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import Image, LaserScan
from soccerbot.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

# Locate ball, opponent, and goals


class Detector:
    def __init__(self):
        self.impub = rospy.Publisher('/soccerbot/image', Image, queue_size=1)
        self.ball_pub = rospy.Publisher('/soccerbot/ball/pose', PoseStamped, queue_size=1)
        self.opponent_pub = rospy.Publisher('/soccerbot/opponent/pose', PoseStamped, queue_size=1)
        self.ball_detected_pub = rospy.Publisher('/soccerbot/ball/image_filtered', Image, queue_size=1)
        self.bridge = CvBridge()
        self.bearing = -1
        self.distance = -1
        self.upper_bound = 70
        self.lower_bound = 200
        self.kernel_size = 7

        # ball detection
        self.ball_hue_upper = 70
        self.ball_hue_lower = 0
        self.ball_sat_lower = 130
        self.ball_val_lower = 130
        self.ball_kernel_size = 5
        self.ball_erode_iterations = 1
        self.ball_dilate_iterations = 1
        self.ball_width_meters = .3 #meters
        
        self.image_topic = rospy.get_param('~image', "/camera/rgb/image_raw")
        rospy.Subscriber(self.image_topic, Image, self.handle_image)
        rospy.Subscriber('/scan', LaserScan, self.handle_scan)

    # Determine ball pose (odom frame) and return it
    def locate_ball(self, im_hsv):

        detected_pixels = np.where( (im_hsv[:, :, 0] <= self.ball_hue_upper) & 
            (im_hsv[:, :, 0] >= self.ball_hue_lower) & 
            (im_hsv[:, :, 1] >= self.ball_sat_lower) & 
            (im_hsv[:, :, 2] >= self.ball_val_lower))


        # Filter out noise using erosion and dilation
        rows, cols = detected_pixels
        kernel = np.ones((self.ball_kernel_size, self.ball_kernel_size), np.uint8)
        im_hsv[rows, cols] = (255, 0, 0)
        im_mono = cv2.cvtColor(im_hsv, cv2.HSV2GRAY)
        im_mono = cv2.erode(im_mono, kernel, iterations = self.ball_erode_iterations)
        im_mono = cv2.dilate(im_mono, kernel, iterations = self.ball_dilate_iterations)
        
        self.ball_detected_pub.publish(self.bridge.cv2_to_imgmsg(im_mono, "bgr8"))

        if len(cols) >= 600: #passes the test
            sum = 0
            for i in cols:
                sum = sum + i
            avg = sum / len(c)
            #im_hsv[:, int(avg)] = (0, 255, 0)
            bearing = int(avg)
        else:
            bearing = -1

        self.ball_width_cols = cols # Not sure if this is correct
        self.ball_bearing = bearing

        

    def locate_opponent(self, image_hsv):
        print("locating red flag")
        
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

        # Blur image
        #kernel = np.ones((self.kernel_size, self.kernel_size), np.float32) / (self.kernel_size * self.kernel_size)
        im_hsv_blurred = cv2.GaussianBlur(im_hsv, (self.kernel_size, self.kernel_size), 0)
        
        ball_bearing = self.locate_ball(im_hsv_blurred)
        
        # Publish filtered image of ball
        self.impub.publish(self.bridge.cv2_to_imgmsg(image, "mono8"))

        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image.

    def handle_scan(self, msg):
        if self.ball_bearing >= 0:
            dist = msg.ranges[-self.bearing]
            if not math.isnan(dist):
                self.ball_distance = dist
            else:
                self.ball_distance = -1
        else:
            self.ball_distance = -1
        #If the bearing is valid, store the corresponding range
        #in self.distance. Decide what to do if range is NaN.

    def build_pose_stamped(self, bearing, distance):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        
        # Convert bearing and distance to Pose in camera_frame
        meters_per_pixel = self.ball_width_meters / self.ball_width_cols
        ball_disp = (320 - bearing) * meters_per_pixel
        ball_theta = math.asin(ball_disp / distance)
        ball_x = math.cos(ball_theta) * distance
        ball_y = math.sin(ball_theta) * distance
        

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            ball_pose = PoseStamped()
            

            location = BallLocation()
            location.bearing = self.bearing
            location.distance = self.distance
            self.locpub.publish(location)
         

rospy.init_node('soccerbot_object_finder')
detector = Detector()
detector.start()
