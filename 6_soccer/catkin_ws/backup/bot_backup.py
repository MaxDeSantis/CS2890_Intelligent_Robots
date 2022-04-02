#!/usr/bin/env python3

from tracemalloc import stop
from turtle import position
from catkin_ws.src.soccerbot.scripts.velocity_manager import VelocityManager
import roslib
import rospy
import numpy as np
import math
import enum
from soccerbot.msg import BallLocation
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import robot_pid



class SoccerBot:

    class RobotState(enum.Enum):
        stop                    = 0,
        search                  = 1,
        approach_ball           = 2,
        approach_intermediate   = 3,
        approach_final          = 4,
        face_ball               = 5,
        kick                    = 6
   
    class RobotSide(enum.Enum):
        side_left       = 1,
        side_right      = 2

    def __init__(self):
        # Robot state tracking
        self.state = self.RobotState.state_stop
        self.wait_time = [None, None]
        self.waiting = False
        self.wait_delay = 1.5
        self.missed_measurements = 0
        self.max_missed_measurements = 5
        self.measure_low_pass_gain = 0.2

        # PID control clamp limits
        self.bearing_control_max = 1.5
        self.bearing_control_min = -1.5
        self.dist_control_upper = .7
        self.dist_control_lower = -.1

        # PID init and gains
        self.bearing_pid = robot_pid.PID(0.005, 0.0, .002, self.bearing_control_max, -self.bearing_control_max)
        self.distance_pid = robot_pid.PID(-0.25, 0, 0.04, self.dist_control_upper, self.dist_control_lower)
        
        # Measured values
        self.dist_measured = 0
        self.bearing_measured = 0
        self.last_approached_side = self.RobotSide.side_left

        # Desired values
        self.dist_setpoint = 1.0
        self.bearing_setpoint = 320

        # Error thresholds to enter KICK state
        self.dist_acceptable_error = .1
        self.bearing_acceptable_error = 10
        self.kick_duration = 1.5


        self.velManager = VelocityManager()

        self.setpoint_pub = rospy.Publisher('/soccerbot/setpoint', BallLocation, queue_size=1)
        self.filtered_location_pub = rospy.Publisher('/soccerbot/filtered_location', BallLocation, queue_size=1)

        # IO - subscribe to measurements and bumper, publish movements
        self.motor_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('/soccerbot/ball_location', BallLocation, self.HandleBallLocation)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.HandleBumped)
    
    # Update internal trackers of ball location for FSM's use
    def HandleBallLocation(self, msg): 

        # New distance measurement, send through low pass filter
        self.prev_distance_measured = self.dist_measured
        self.dist_measured = (msg.distance * self.measure_low_pass_gain) + self.prev_distance_measured * (1 - self.measure_low_pass_gain)
        self.last_distance_time = rospy.Time.now()

        # New bearing measurement, low pass filter
        self.prev_bearing_measured = self.bearing_measured
        self.bearing_measured = (msg.bearing * self.measure_low_pass_gain) + self.prev_bearing_measured * (1 - self.measure_low_pass_gain)
        self.last_bearing_time = rospy.Time.now()

        
    # If bumped, transition to stop state
    def HandleBumped(self, msg):
        print("bumped!")
        self.state = self.RobotState.state_stop
        self.waiting = False # Continue to reset wait time until bumping is stopped


    # ----------------------------------------------------------------------------------------
    #
    # Default Twists
    #
    # ----------------------------------------------------------------------------------------

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
        kick_twist.linear.x = 1.0
        kick_twist.angular.z = 0
        return kick_twist
    
    def TwistKickStart(self):
        kick_twist = Twist()
        kick_twist.linear.x = 0.7
        kick_twist.linear.z = 0
        return kick_twist

    # ----------------------------------------------------------------------------------------
    #
    # PID Controls
    #
    # ----------------------------------------------------------------------------------------

    # Compute control value for bearing and determine if it is within margin of error. Return results.
    def ControlBearing(self):
        print("Centering")

        ang_control = self.prev_twist.angular.z
        accept_ang = False

        if self.bearing_measured > 0 and self.bearing_measured < 640:
            ang_control = self.bearing_pid.GetControl(self.bearing_setpoint, self.bearing_measured, rospy.Time.now())

        if abs(self.bearing_setpoint - self.bearing_measured) < self.bearing_acceptable_error:
            accept_ang = True

        return (ang_control, accept_ang)

    # Compute control value for distance and determine if it is within margin of error. Return results.
    def ControlPosition(self):
        print("Positioning")
        dist_control = self.prev_twist.linear.x
        accept_dist = False

        if self.dist_measured > 0: # Valid distance, approach setpoint
            dist_control = self.distance_pid.GetControl(self.dist_setpoint, self.dist_measured, rospy.Time.now())

        if abs(self.dist_measured - self.dist_setpoint) < self.dist_acceptable_error:
            accept_dist = True
        
        return (dist_control, accept_dist)

    def ComputeNextPositions(self):
        print("computing")


    # ----------------------------------------------------------------------------------------
    #
    # Robot Behaviors
    #
    # ----------------------------------------------------------------------------------------

    # STOP
    def StoppedBehavior(self):
        print("STOPPED")
        self.wait_time[1] = rospy.Time.now()
        # Start clock
        if not self.waiting:
            self.wait_time[0] = rospy.Time.now()
            self.waiting = True
            self.next_twist = self.TwistStopped()
        else:       
            if (self.wait_time[1] - self.wait_time[0]).to_sec() < self.wait_delay:     # Remain stopped if still waiting
                self.next_twist = self.TwistStopped()
                
            else:                                               # Transition to searching if time has passed
                self.waiting = False
                self.state = self.RobotState.search

    # SEARCH
    def SearchBehavior(self):
        print("SEARCH")
        self.next_twist = self.TwistSearch()
        
        # Begin approach if ball location is valid
        if self.dist_measured > 0 and self.bearing_measured > 0 and self.bearing_measured < 640:
            self.state = self.RobotState.approach_ball

    # APPROACH BALL
    def ApproachBallBehavior(self):
        print("APPROACH")

        # Compute and acuate controls. Move to next state if errors acceptable (control functions return true)

        self.bearing_setpoint = 320
        self.dist_setpoint = 1.5

        (angular_control, acceptable_bearing)   = self.ControlBearing()
        (distance_control, acceptable_distance) = self.ControlPosition()
        self.next_twist.angular.z = angular_control
        self.next_twist.linear.x = distance_control

        # Compute new points and approach them if in appropriate position
        if acceptable_bearing and acceptable_distance:
            self.ComputeNextPositions()
            self.state = self.RobotState.approach_intermediate

        # Handle losing vision of the ball while approaching
        if self.bearing_measured < 0 or self.bearing_measured > 640 or self.dist_measured < 0:
            if self.missed_measurements < self.max_missed_measurements:
                self.missed_measurements += 1
            else:
                self.missed_measurements = 0
                self.state = self.RobotState.search


        self.last_approached_side = self.RobotSide.side_right if (self.bearing_measured > 320) else self.RobotSide.side_left
        
    
    # APPROACH INTERMEDIATE
    def ApproachIntermediateBehavior(self):
        print("INTERMEDIATE")

    # APPROACH FINAL
    def ApproachFinalBehavior(self):
        print("FINAL")
    
    # TURN
    def FaceBallBehavior(self):
        print("TURN")

        # Turn robot until ball is centered in view with minimal error.

        if self.bearing_measured < 0:   # Ball not in view, begin turning
            self.next_twist = self.TwistSearch()
        elif self.bearing_measured > 0 and self.bearing_measured < 640: # Ball in view, use PID
            bearing_control = self.bearing_pid.GetControl(self.bearing_setpoint, self.bearing_measured, rospy.Time.now())
            self.next_twist.angular.z = bearing_control
            self.next_twist.linear.x = 0




    # KICK
    def KickBehavior(self):
        print("KICK")

        self.next_twist = self.TwistKick()
        self.wait_time[1] = rospy.Time.now()

        if not self.waiting:
            self.wait_time[0] = rospy.Time.now()
            self.waiting = True
        else:
            if (self.wait_time[1] - self.wait_time[0]).to_sec() > self.kick_duration:
                self.waiting = False
                self.state = self.RobotState.stop


    # ----------------------------------------------------------------------------------------
    #
    # Main State Machine
    #
    # ----------------------------------------------------------------------------------------

    def Run(self):
        rate = rospy.Rate(10)
        self.next_twist = Twist()
        
        # STOP -> SEARCH -> APPROACH BALL -> APPROACH INTERMEDIATE -> APPROACH FINAL -> TURN -> KICK
        while not rospy.is_shutdown():
            
            if self.state == self.RobotState.stop:                      # Stop robot in case of bumping, not required but desireable
                self.StoppedBehavior()
            
            elif self.state == self.RobotState.search:                  # Search for ball by rotating in place. Check location, if acceptable switch to approach.
                self.SearchBehavior()
            
            elif self.state == self.RobotState.approach_ball:           # Dual PID to control bearing and distance. If controls out of bounds, return to search.
                self.RobotApproach()

            elif self.state == self.RobotState.approach_intermediate:   # Approach position 1.5 meters to side of robot using odometry
                self.ApproachIntermediateBehavior()

            elif self.state == self.RobotState.approach_final:          # Approach position 1.5 meters behind robot
                self.ApproachFinalBehavior()
            
            elif self.state == self.RobotState.face_ball:               # Turn and face ball
                self.FaceBallBehavior()

            elif self.state == self.RobotState.kick:                    # Drive towards ball at 1 m/s for 1.5 seconds, or until bump sensor detected
                self.RobotKick()

            else:
                print("UNKNOWN STATE")
            
            
            #self.next_twist.linear.x = 0 # don't move forward right now
            self.motor_pub.publish(self.next_twist)
            self.prev_twist = self.next_twist
            
            sp = BallLocation()
            sp.bearing = self.bearing_setpoint
            sp.distance = self.dist_setpoint

            fb = BallLocation()
            fb.bearing = self.bearing_measured
            fb.distance = self.dist_measured
            
            self.setpoint_pub.publish(sp)
            self.filtered_location_pub.publish(fb)
            
            rate.sleep()