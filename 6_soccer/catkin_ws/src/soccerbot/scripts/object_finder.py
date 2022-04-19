#!/usr/bin/env python3
import roslib
import rospy
import tf2_ros
import tf2_geometry_msgs
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
        self.ball_bearing = -1
        self.ball_distance = -1
        self.ball_stamp = -1
        self.ball_stamp_prev = -1
        self.upper_bound = 70
        self.lower_bound = 200
        self.kernel_size = 57

        # ball detection
        self.ball_hue_upper = 40
        self.ball_hue_lower = 0
        self.ball_sat_lower = 130
        self.ball_val_lower = 130
        self.ball_kernel_size = 25
        self.ball_erode_iterations = 1
        self.ball_dilate_iterations = 1
        self.ball_width_meters = .3 #meters
        self.ball_pose_odom_prev = PoseStamped()
        
        self.image_topic = rospy.get_param('~image', "/local_image")
        rospy.Subscriber(self.image_topic, Image, self.handle_image)
        rospy.Subscriber('/scan', LaserScan, self.handle_scan)
        
        # Transform listeners
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Determine ball bearing and min/max bounding values
    def locate_ball(self, im_hsv):

        # Find pixels of correct color
        detected_pixels = np.where( (im_hsv[:, :, 0] <= self.ball_hue_upper) & 
            (im_hsv[:, :, 0] >= self.ball_hue_lower) & 
            (im_hsv[:, :, 1] >= self.ball_sat_lower) & 
            (im_hsv[:, :, 2] >= self.ball_val_lower))


        # Filter out noise using erosion and dilation
        rows, cols = detected_pixels
        kernel = np.ones((self.ball_kernel_size, self.ball_kernel_size), np.uint8)
        im_hsv[rows, cols] = (255, 0, 0)
        im_bgr_intermediate = cv2.cvtColor(im_hsv, cv2.COLOR_HSV2BGR)
        im_mono = cv2.cvtColor(im_bgr_intermediate, cv2.COLOR_BGR2GRAY)
        im_mono = cv2.erode(im_mono, kernel, iterations = self.ball_erode_iterations)
        im_mono = cv2.dilate(im_mono, kernel, iterations = self.ball_dilate_iterations)
        
        # With filtered blob, we can be reasonably confident that there is minimal noise.
        minV = maxV = -1
        if len(cols) >= 200: #passes the test
            sum = 0
            minV = cols[0]
            maxV = cols[0]
            for i in cols:
                sum = sum + i
                if i > maxV:
                    maxV = i
                elif i < minV:
                    minV = i
            avg = sum / len(cols)
            im_mono[:, int(avg)] = 0
            im_mono[:, int(maxV)] = 255
            im_mono[:, int(minV)] = 255
            bearing = int(avg)
        else:
            bearing = -1

        self.ball_width_cols = maxV - minV

        self.ball_bearing = bearing
        self.ball_detected_pub.publish(self.bridge.cv2_to_imgmsg(im_mono, "mono8"))

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
        im_hsv_blurred = cv2.GaussianBlur(im_hsv, (self.kernel_size, self.kernel_size), 0)
        
        ball_bearing = self.locate_ball(im_hsv_blurred)
        
        # Publish filtered image of ball
        self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


    def handle_scan(self, msg):
        if self.ball_bearing >= 0:
            dist = msg.ranges[-self.ball_bearing]
            if not math.isnan(dist):
                self.ball_distance = dist
                self.ball_stamp = msg.header.stamp
            else:
                self.ball_distance = -1
        else:
            self.ball_distance = -1

    def build_pose_stamped(self, bearing, distance, stamp):
        pose = PoseStamped()
        pose.header.frame_id = "camera_depth_frame"
        pose.header.stamp = stamp
        
        # 62 deg fov
        if self.ball_width_cols > 0 and distance > 0 and 0 <= bearing <= 640:
            # Convert bearing and distance to Pose in camera_frame
            
            ball_angle = (320 - bearing) * (62 / 640) * (math.pi / 180) # 62 degrees for 640 pixels, pi radians per degree.
            #ball_horizontal_disp = distance * asin(ball_angle)
            
            #print("disp: ", ball_disp, "mpp: ", meters_per_pixel, "theta: ", ball_theta * 180/math.pi, "cols: ", self.ball_width_cols)
            pose.pose.position.x = math.cos(ball_angle) * distance
            pose.pose.position.y = math.sin(ball_angle) * distance
            pose.pose.position.z = 0
            
            # Transform to odom frame
            try:
                transform = self.tf_buffer.lookup_transform("odom", pose.header.frame_id, pose.header.stamp, rospy.Duration(1.0))
                odomPose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                self.ball_pose_odom_prev = odomPose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                odomPose = self.ball_pose_odom_prev
        else:
            odomPose = self.ball_pose_odom_prev
            
        
        return odomPose
        
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            if not self.ball_stamp == self.ball_stamp_prev:
                ball_pose = PoseStamped()
                ball_pose = self.build_pose_stamped(self.ball_bearing, self.ball_distance, self.ball_stamp)
                self.ball_pub.publish(ball_pose)
                self.ball_stamp_prev = self.ball_stamp

            #location = BallLocation()
            #location.bearing = self.bearing
            #location.distance = self.distance
            #self.locpub.publish(location)


rospy.init_node('soccerbot_object_finder')
detector = Detector()
detector.start()
