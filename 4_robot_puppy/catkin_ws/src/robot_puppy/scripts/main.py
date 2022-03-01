#!/usr/bin/env python3

import roslib
import rospy
import numpy as np
import math
import enum
from robot_puppy.msg import BallLocation
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


class PID:
    def __init__(self, kp, ki, kd, max, min):
        # set gains from rosparams?
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = rospy.Time.now()
        self.max = max
        self.min = min

    def GetControl(self, setpoint, measured, current_time):
        # Compute control, return
        time_diff = (current_time - self.prev_time).to_sec()

        error = float(setpoint - measured)
        derivative = (error - self.prev_error) / time_diff
        self.integral += error
        
        print("set: ", setpoint, " error: ", error)

        control = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Clamp output and handle integral windup
        if control > self.max:
            control = min(control, self.max)
            self.integral -= error
        elif control < self.min:
            control = max(control, self.min)
            self.integral -= error

        self.prev_error = error
        self.prev_time = current_time

        return control

class Puppy:

    class RobotState(enum.Enum):
        state_search    = 1,
        state_approach  = 2,
        state_kick      = 3,
        state_stop      = 4
   
    class RobotSide(enum.Enum):
        side_left       = 1,
        side_right      = 2

    def __init__(self):
        self.state = self.RobotState.state_stop
        self.wait_time = [None, None]
        self.waiting = False

        self.bearing_control_max = .7
        self.dist_control_max = .2

        self.bearing_pid = PID(0.002, 0, .001, self.bearing_control_max, -self.bearing_control_max)
        self.distance_pid = PID(0.15, 0, .1, self.dist_control_max / 2.0, -self.dist_control_max)
        
        self.dist_measured = 0
        self.bearing_measured = 0
        self.last_approached_side = self.RobotSide.side_left

        self.dist_setpoint = .6
        self.bearing_setpoint = 320

        self.dist_acceptable_error = .4
        self.bearing_acceptable_error = 20
        
        self.wait_delay = 3.0
        self.missed_cycles = 0
        

        self.motor_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        #self.measured = rospy.Publisher('/puppy/controls', BallLocation, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/puppy/setpoint', BallLocation, queue_size=1)
        rospy.Subscriber('/puppy/ball_location', BallLocation, self.HandleBallLocation)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.HandleBumped)
    
    # Update internal trackers of ball location for FSM's use
    def HandleBallLocation(self, msg):
        # do something
        self.dist_measured = msg.distance
        self.last_distance_time = rospy.Time.now()

        self.bearing_measured = msg.bearing
        self.last_bearing_time = rospy.Time.now()
        

    
    # If bumped, transition to stop state
    def HandleBumped(self, msg):
        print("bumped!")
        self.state = self.RobotState.state_stop

    def TwistStopped(self):
        stopped_twist = Twist()
        stopped_twist.angular.z = 0
        stopped_twist.linear.x = 0
        return stopped_twist

    def TwistSearch(self):
        search_twist = Twist()
        search_twist.linear.x = 0
        search_twist.angular.z = .8
        
        if self.last_approached_side == self.RobotSide.side_right:
            search_twist.angular.z *= -1
            
        return search_twist

    def TwistKick(self):
        kick_twist = Twist()
        kick_twist.linear.x = 1.2
        kick_twist.angular.z = 0
        return kick_twist

    def Run(self):
        # Get moving
        rate = rospy.Rate(10)
        twist = Twist()
        
        while not rospy.is_shutdown():

            # Stop robot in case of bumping, not required but desireable
            if self.state == self.RobotState.state_stop:
                print("STOPPED")
                self.wait_time[1] = rospy.Time.now()
                # Start clock
                if not self.waiting:
                    self.wait_time[0] = rospy.Time.now()
                    self.waiting = True
                    twist = self.TwistStopped()
                else:       
                    if (self.wait_time[1] - self.wait_time[0]).to_sec() < self.wait_delay:     # Remain stopped if still waiting
                        twist = self.TwistStopped()
                        
                    else:                                               # Transition to searching if time has passed
                        self.waiting = False
                        self.state = self.RobotState.state_search
            
            # Search for ball by rotating in place. Check location, if acceptable switch to approach.
            elif self.state == self.RobotState.state_search:
                print("SEARCH")
                twist = self.TwistSearch()
                
                # Begin approach if ball location is valid
                if self.dist_measured > 0 and self.bearing_measured > 0 and self.bearing_measured < 640:
                    self.state = self.RobotState.state_approach

            # Dual PID to control bearing and distance. If controls out of bounds, return to search.
            elif self.state == self.RobotState.state_approach:
                print("APPROACH")
                
                if self.dist_measured > 0 and self.bearing_measured > 0 and self.bearing_measured < 640:
                    bearing_control = self.bearing_pid.GetControl(self.bearing_setpoint, self.bearing_measured, rospy.Time.now())
                    dist_control = -self.distance_pid.GetControl(self.dist_setpoint, self.dist_measured, rospy.Time.now())
                    
                    print("bc: ", bearing_control, " dc: ", dist_control)
                    print("dist: ", self.dist_measured, " bear: ", self.bearing_measured)
                            
                    twist.angular.z = bearing_control
                    twist.linear.x = dist_control
                    
                    # Switch to search if controls out of bounds, set twist to 0.
                    if abs(dist_control) > self.dist_control_max or abs(bearing_control > self.bearing_control_max):
                        self.state = self.RobotState.state_search
                        twist = self.TwistStopped()
                        
                    
                    # If errors are low enough, enter kick state
                    if abs(self.bearing_setpoint - self.bearing_measured) < self.bearing_acceptable_error and abs(self.dist_setpoint - self.dist_measured) < self.dist_acceptable_error:
                        self.state = self.RobotState.state_kick
                    
                    self.last_approached_side = self.RobotSide.side_right if (self.bearing_measured > 320) else self.RobotSide.side_left
                    
                elif self.missed_cycles < 10:
                    twist = self.prev_twist
                    self.missed_cycles += 1
                else:
                    self.missed_cycles = 0
                    self.state = self.RobotState.state_search

            # Drive towards ball at 1 m/s for 1.5 seconds, or until bump sensor detected
            elif self.state == self.RobotState.state_kick:
                print("KICK")

                twist = self.TwistKick()
                self.wait_time[1] = rospy.Time.now()

                if not self.waiting:
                    self.wait_time[0] = rospy.Time.now()
                    self.waiting = True
                else:
                    if (self.wait_time[1] - self.wait_time[0]).to_sec() > 1.5:     # If 1.5s have passed, begin searching again
                        self.waiting = False
                        self.state = self.RobotState.state_search

            else:
                print("UNKNOWN STATE")
            

            self.motor_pub.publish(twist)
            self.prev_twist = twist
            
            sp = BallLocation()
            sp.bearing = self.bearing_setpoint
            sp.distance = self.dist_setpoint
            
            self.setpoint_pub.publish(sp)
            
            rate.sleep()



rospy.init_node('puppy_main')
robo_pup = Puppy()
robo_pup.Run()
