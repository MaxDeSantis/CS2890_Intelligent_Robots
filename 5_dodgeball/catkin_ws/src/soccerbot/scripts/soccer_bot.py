#!/usr/bin/env python3

from tracemalloc import stop
from turtle import position

from cv2 import sqrt
import roslib
import rospy
import numpy as np
import math
import enum
from soccerbot.msg import BallLocation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from kobuki_msgs.msg import BumperEvent
import robot_pid
import velocity_manager


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
        self.state = self.RobotState.stop
        self.wait_time = [None, None]
        self.waiting = False
        self.wait_delay = 1.5
        self.missed_measurements = 0
        self.max_missed_measurements = 5
        self.measure_low_pass_gain = 0.2
        self.max_angle_err_before_moving = math.pi / 6.0
        

        # PID control clamp limits
        self.bearing_control_max = 1.5
        self.bearing_control_min = -1.5
        self.dist_control_upper = 1.0
        self.dist_control_lower = -.2
        self.max_acc_ang        = .3
        self.max_acc_lin        = .05

        # PID init and gains
        
        self.anglePID = robot_pid.PID(0.0056, 0.0, .0016, self.bearing_control_max, -self.bearing_control_max)
        self.rangePID = robot_pid.PID(-0.44, 0.0, 0.2, self.dist_control_upper, self.dist_control_lower)
        
        # Measured values
        self.dist_measured = 0
        self.bearing_measured = 0
        self.last_approached_side = self.RobotSide.side_left

        # Desired values
        self.dist_setpoint = 1.5
        self.bearing_px_setpoint = 320

        # Error thresholds to enter KICK 
        self.dist_acceptable_error = .15
        self.bearing_acceptable_error = 18
        self.angular_vel_acceptable_error = .09
        self.kick_duration = 1.5

        # Velocities
        self.kick_lin_x = 1.3
        self.search_ang_z = 0.8

        self.velManager = velocity_manager.VelocityManager(self.max_acc_ang, self.max_acc_lin)

        # IO - subscribe to measurements and bumper, publish movements
        self.motor_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('/soccerbot/ball_location', BallLocation, self.HandleBallLocation)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.HandleBumped)
        rospy.Subscriber('/odom', Odometry, self.HandlePose)
        
    
    # ----------------------------------------------------------------------------------------
    #
    # ROS Sub callbacks
    #
    # ----------------------------------------------------------------------------------------
    
    # Update internal trackers of ball location for FSM's use
    def HandleBallLocation(self, msg): 

        # Mark as missed if invalid measurement while in approach ball state
        if msg.distance < 0 or msg.bearing < 0 and self.state == self.RobotState.approach_ball:
            self.missed_measurements += 1
            
        self.prev_distance_measured = self.dist_measured
        self.dist_measured = (msg.distance * self.measure_low_pass_gain) + self.prev_distance_measured * (1 -self.measure_low_pass_gain)
        self.last_distance_time = rospy.Time.now()


        # New bearing measurement, low pass filter
        self.prev_bearing_measured = self.bearing_measured
        self.bearing_measured = (msg.bearing * self.measure_low_pass_gain) + self.prev_bearing_measured * (1 - self.measure_low_pass_gain)
        self.last_bearing_time = rospy.Time.now()

    # Update robot's position according to odometry
    def HandlePose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (_, _, self.theta) = euler_from_quaternion(q)
        #self.theta = (self.theta + math.pi) % (2*math.pi)
        
        self.last_ang_vel = msg.twist.twist.angular.z

        
    # If bumped, transition to stop state
    def HandleBumped(self, msg):
        print("bumped!")
        self.state = self.RobotState.stop
        self.waiting = False # Continue to reset wait time until bumping is stopped


    # ----------------------------------------------------------------------------------------
    #
    # PID Control wrappers, intermediate + final position computations
    #
    # ----------------------------------------------------------------------------------------

    # Compute control value for bearing and determine if it is within margin of error. Return results.
    def ControlBearing(self, setpoint, measured):

        ang_control = self.prev_twist.angular.z
        accept_ang = False

        ang_control = self.anglePID.GetControl(setpoint, measured, rospy.Time.now())

        if abs(setpoint - measured) < self.bearing_acceptable_error:
            accept_ang = True
            ang_control = 0

        return (ang_control, accept_ang)
        

    # Compute control value for distance and determine if it is within margin of error. Return results.
    def ControlPosition(self, setpoint, measured):

        dist_control = self.prev_twist.linear.x
        accept_dist = False
        
        dist_control = self.rangePID.GetControl(setpoint, measured, rospy.Time.now())

        if abs(setpoint - measured) < self.dist_acceptable_error:
            accept_dist = True
            dist_control = 0
        
        return (dist_control, accept_dist)
        
    # Gets angle within -pi to pi, with correct direction and smaller side
    def GetCorrectAngleDiff(self, desired, actual):
        angle_diff = (actual - desired)

        if angle_diff > math.pi:
            angle_diff = (angle_diff) % (-2*math.pi)
        elif angle_diff < -math.pi:
            angle_diff = (angle_diff) % (2*math.pi)
        
        return angle_diff

    def ComputeNextPositions(self):
        #print("computing")
        final_x = self.x + 3.0 * math.cos(self.theta)
        final_y = self.y + 3.0 * math.sin(self.theta)
        self.final_goal = (final_x, final_y)

        intermediate_x = self.x + (self.dist_measured * math.sqrt(2.0)) * math.cos(self.theta - math.pi/4.0)
        intermediate_y = self.y + (self.dist_measured * math.sqrt(2.0)) * math.sin(self.theta - math.pi/4.0)
        self.intermediate_goal = (intermediate_x, intermediate_y)
        
        self.original_detected_angle = self.theta

        if self.theta >= 0:
            self.final_turn_angle = self.theta - math.pi
        else:
            self.final_turn_angle = self.theta + math.pi
            
        print("original angle: ", self.theta)
        print("reverse. original angle: ", self.final_turn_angle)

    def GetVector(self, from_x, from_y, to_x, to_y):
        bearing = math.atan2(to_y - from_y, to_x - from_x)
        distance = math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
        return (bearing, distance)


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
            self.velManager.SetDesiredVelocity(0, 0)

        else:       
            if (self.wait_time[1] - self.wait_time[0]).to_sec() < self.wait_delay:     # Remain stopped if still waiting
                self.velManager.SetDesiredVelocity(0, 0)
                
            else:                                               # Transition to searching if time has passed
                self.waiting = False
                self.state = self.RobotState.search

    # SEARCH
    def SearchBehavior(self):
        print("SEARCH")
        self.velManager.SetDesiredVelocity(self.search_ang_z, 0)
        
        # Begin approach if ball location is valid
        if self.dist_measured > 0 and self.bearing_measured > 0 and self.bearing_measured < 640:
            self.state = self.RobotState.approach_ball

    # APPROACH BALL
    def ApproachBallBehavior(self):
        print("APPROACH")

        # Compute and acuate controls. Move to next state if errors acceptable (control functions return true)

        (angular_control, acceptable_bearing)   = self.ControlBearing(self.bearing_px_setpoint, self.bearing_measured)
        (distance_control, acceptable_distance) = self.ControlPosition(self.dist_setpoint, self.dist_measured)
        self.velManager.SetDesiredVelocity(angular_control, distance_control)

        # Compute new points and approach them if in appropriate position
        if acceptable_bearing and acceptable_distance:
            self.ComputeNextPositions()
            print("intermediate: ", self.intermediate_goal)
            print("final: ", self.final_goal)
            self.state = self.RobotState.approach_intermediate

        # Handle losing vision of the ball while approaching. Don't call if reached accepetable bounds
        elif self.missed_measurements > self.max_missed_measurements:
            self.missed_measurements = 0
            self.state = self.RobotState.search
            self.velManager.SetDesiredVelocity(0, 0)

        if  self.bearing_measured > 0 and self.bearing_measured < 640:
            self.last_approached_side = self.RobotSide.side_right if (self.bearing_measured > 320) else self.RobotSide.side_left
        
        print("Desired Angle: ", 320, " | Self angle: ", self.bearing_measured," | Ang ctrl: ", angular_control, " | Desired dist: ", self.dist_setpoint, " | Meas. Dist: ", self.dist_measured)
    
    # APPROACH INTERMEDIATE
    def ApproachIntermediateBehavior(self):
        print("INTERMEDIATE")
        

        (angle_at_point, range_to_point) = self.GetVector(self.x, self.y, self.intermediate_goal[0], self.intermediate_goal[1])
        
        measured_angle_diff = self.GetCorrectAngleDiff(angle_at_point, self.theta) * (640.0/math.pi)
        
        (angle_control, accept_ang) = self.ControlBearing(0, measured_angle_diff)
        (range_control, accept_range) = self.ControlPosition(0, range_to_point)

        #print("Desired Angle: ", 0, " | angle diff: ", measured_angle_diff,  " | Range: ", range_to_point, " | Ang ctrl: ", angle_control, " | Range ctrl: ", range_control)
        
        if abs(angle_at_point - self.theta) > self.max_angle_err_before_moving:
            range_control = 0
            
        self.velManager.SetDesiredVelocity(angle_control, range_control)
        print("range?: ", accept_range, "| angle?: ", accept_ang)
        if accept_range and accept_ang:
            self.state = self.state.approach_final

    # APPROACH FINAL
    def ApproachFinalBehavior(self):
        print("FINAL")

        (angle_at_point, range_to_point) = self.GetVector(self.x, self.y, self.final_goal[0] , self.final_goal[1] )

        measured_angle_diff = self.GetCorrectAngleDiff(angle_at_point, self.theta) * (640.0/math.pi)
            
        (angle_control, accept_ang) = self.ControlBearing(0, measured_angle_diff)
        (range_control, accept_range) = self.ControlPosition(0, range_to_point)
        
        if abs(angle_at_point - self.theta) > self.max_angle_err_before_moving:
            range_control = 0
        
        #print("Desired Angle: ", 0, " | angle diff: ", measured_angle_diff, " | Range: ", range_to_point, " | Ang ctrl: ", angle_control, " | Range ctrl: ", range_control)
                
        self.velManager.SetDesiredVelocity(angle_control, range_control)
        print("range?: ", accept_range, "| angle?: ", accept_ang)
        if accept_range and accept_ang:
            self.state = self.state.face_ball
    
    # TURN
    def FaceBallBehavior(self):
        print("TURN")

        # Turn to face original direction if don't see ball
       
        if self.bearing_measured < 0 or self.bearing_measured > 640:   # Ball not in view, begin turning
            #self.velManager.SetDesiredVelocity(self.search_ang_z, 0)
            print("undetected in turn")
            measured_angle_diff = self.GetCorrectAngleDiff(self.final_turn_angle, self.theta) * (640.0/math.pi)
            (bearing_control, accept_angle) = self.ControlBearing(0, measured_angle_diff)
            
            self.velManager.SetDesiredVelocity(bearing_control, 0)
            print("Desired: ", -self.original_detected_angle, " | Act: ", self.theta, " | Orig ang diff: ", (self.theta - self.final_turn_angle), " | new ang diff: ", measured_angle_diff)
            
            # check if ball in view at proper angle
            if accept_angle and (self.bearing_measured < 0 or self.bearing_measured > 640):
                # ball not in view, return to search
                self.state = self.state.search
                
        else: # Ball in view, use PID
            (bearing_control, accept_angle) = self.ControlBearing(self.bearing_px_setpoint, self.bearing_measured)
            #(range_control, accept_range)   = self.ControlPosition(1, self.dist_measured)
            print("Desired Angle: ", 320, " | Self angle: ", self.bearing_measured," | Ang ctrl: ", bearing_control, " | Desired dist: ", self.dist_setpoint, " | Meas. Dist: ", self.dist_measured)
                    
            self.velManager.SetDesiredVelocity(bearing_control, 0)
            
            print("| angle?: ", accept_angle, " : ", self.bearing_measured)
            if accept_angle and abs(self.last_ang_vel) <= self.angular_vel_acceptable_error:
                self.state = self.RobotState.kick


    # KICK
    def KickBehavior(self):
        print("KICK")

        self.velManager.SetDesiredVelocity(0, self.kick_lin_x)
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
        
        # STOP -> SEARCH -> APPROACH BALL -> APPROACH INTERMEDIATE -> APPROACH FINAL -> TURN -> KICK
        while not rospy.is_shutdown():
            
            if self.state == self.RobotState.stop:                      # Stop robot in case of bumping, not required but desireable
                self.StoppedBehavior()
            
            elif self.state == self.RobotState.search:                  # Search for ball by rotating in place. Check location, if acceptable switch to approach.
                self.SearchBehavior()
            
            elif self.state == self.RobotState.approach_ball:           # Dual PID to control bearing and distance. If controls out of bounds, return to search.
                self.ApproachBallBehavior()

            elif self.state == self.RobotState.approach_intermediate:   # Approach position 1.5 meters to side of robot using odometry
                self.ApproachIntermediateBehavior()

            elif self.state == self.RobotState.approach_final:          # Approach position 1.5 meters behind robot
                self.ApproachFinalBehavior()
            
            elif self.state == self.RobotState.face_ball:               # Turn and face ball
                self.FaceBallBehavior()

            elif self.state == self.RobotState.kick:                    # Drive towards ball at 1 m/s for 1.5 seconds, or until bump sensor detected
                self.KickBehavior()

            else:
                print("UNKNOWN STATE")
                
            newTwist = Twist()
            newTwist = self.velManager.GetNextTwist()

            self.motor_pub.publish(newTwist)
            
            self.prev_twist = newTwist
            
            
            rate.sleep()
