#!/usr/bin/env python3
# Author: Max DeSantis
# Purpose: CS2890 - Intelligent Robots - Lab 2 - Prison Break

import rospy
import enum
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class Robot:
    # Track current state of the bot
    class RobotState(enum.Enum):
        moving_forward = 1
        moving_backwards = 2
        turning = 3
    
    # Setup state and ROS
    def __init__(self):
        self.move_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
        self.current_state = self.RobotState.moving_forward
        self.debug = rospy.get_param('~debug', False)
        self.waiting = False
        self.run()
    
    # Change state if bumped
    def bumped(self, msg):
        if self.debug: print("BUMPED")
        self.current_state = self.RobotState.moving_backwards

    
    # Actuate according to current state
    def run(self):
        rate = rospy.Rate(10)
        twist = Twist()
        start_time = 0
        
        
        while not rospy.is_shutdown():
        
            # Move forwards until bumped
            if self.current_state == self.RobotState.moving_forward:
                if self.debug: print("FORWARD")
                twist.angular.z = 0
                twist.linear.x = .1
                
            # Once bumped, move backwards for two seconds.
            elif self.current_state == self.RobotState.moving_backwards:
                if self.debug: print("BACKWARDS")

                if not self.waiting:
                    start_time = rospy.Time.now() # Measure when bot began backing up
                    self.waiting = True
                    twist.linear.x = -.1
                    twist.angular.z = 0
                else:
                    if rospy.Time.now().secs - start_time.secs < 2:
                        twist.linear.x = -.1
                        twist.angular.z = 0
                    else:
                        self.waiting = False
                        self.current_state = self.RobotState.turning
                        
            # Turn at some speed for some amount of time. can make random for fun.
            elif self.current_state == self.RobotState.turning:
                if self.debug: print("TURNING")
                
                if not self.waiting:
                    start_time = rospy.Time.now()
                    self.waiting = True
                    twist.linear.x = 0
                    twist.angular.z = 1
                else:
                    if rospy.Time.now().secs - start_time.secs < 2:
                        twist.linear.x = 0
                        twist.angular.z = 1
                    else:
                        self.waiting = False
                        self.current_state = self.RobotState.moving_forward
                
                
            self.move_pub.publish(twist)
            rate.sleep()


rospy.init_node('prison_break')
robot = Robot()
