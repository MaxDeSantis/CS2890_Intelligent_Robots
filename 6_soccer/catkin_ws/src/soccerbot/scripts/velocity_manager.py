import rospy

from geometry_msgs.msg import Twist
import robot_pid

import game_state
import parameter_manager

# Receive velocity setpoints and control robot.
# - Use velocity setpoints to actuate robot -> implement some form of acceleration clamping

class VelocityManager:

    def __init__(self, parameter_manager, game_state):
        # State instances from soccerbot_main
        self.parameterManager   = parameter_manager
        self.gameState          = game_state

        # PID
        self.ballBearingPID     = robot_pid.PID(self.parameterManager.BEARING_KP,
                                    self.parameterManager.BEARING_KI,
                                    self.parameterManager.BEARING_KD,
                                    self.parameterManager.BEARING_MAX,
                                    self.parameterManager.BEARING_MIN)

        self.rangePID           = robot_pid.PID(self.parameterManager.RANGE_KP,
                                    self.parameterManager.RANGE_KI,
                                    self.parameterManager.RANGE_KD,
                                    self.parameterManager.RANGE_MAX,
                                    self.parameterManager.RANGE_MIN)

        self.thetaPID           = robot_pid.PID(self.parameterManager.THETA_KP,
                                    self.parameterManager.THETA_KI,
                                    self.parameterManager.THETA_KD,
                                    self.parameterManager.MAX_ANGULAR_VELOCITY,
                                    -self.parameterManager.MAX_ANGULAR_VELOCITY)
        
        self.cmdVel             = Twist()
        
        self.prev_lin_x = 0
        self.prev_ang_z = 0

    # Returns angular z and linear x clamped to ensure acceleration is within maximums.
    def LimitAcceleration(self, desired_angular_z_vel, desired_linear_x_vel):
        # Clamp angular
        if desired_angular_z_vel - self.prev_ang_z > self.parameterManager.MAX_ANGULAR_ACCELERATION:
            # Positive, greater than maximum angular acc.
            limited_angular_z_vel = self.prev_ang_z + self.parameterManager.MAX_ANGULAR_ACCELERATION
        elif desired_angular_z_vel - self.prev_ang_z < -self.parameterManager.MAX_ANGULAR_ACCELERATION:
            # Negative, less than minimum angular acc (negative max)
            limited_angular_z_vel = self.prev_ang_z - self.parameterManager.MAX_ANGULAR_ACCELERATION
        else:
            # Within limits, go ahead
            limited_angular_z_vel = desired_angular_z_vel
        self.prev_ang_z = limited_angular_z_vel
        
        # Clamp linear
        if desired_linear_x_vel - self.prev_lin_x > self.parameterManager.MAX_LINEAR_ACCELERATION:
            limited_linear_x_vel = self.prev_lin_x + self.parameterManager.MAX_LINEAR_ACCELERATION
        elif desired_linear_x_vel - self.prev_lin_x < -self.parameterManager.MAX_LINEAR_ACCELERATION:
            limited_linear_x_vel = self.prev_lin_x - self.parameterManager.MAX_LINEAR_ACCELERATION
        else:
            limited_linear_x_vel = desired_linear_x_vel
        self.prev_lin_x = limited_linear_x_vel
        
        return (limited_angular_z_vel, limited_linear_x_vel)

    # Directly sets output twist - no PID or management
    def SetRawVelocity(self, raw_angular_z_vel, raw_linear_x_vel):
        rawVel = Twist()
        rawVel.angular.z = raw_angular_z_vel
        rawVel.linear.x = raw_linear_x_vel
        self.cmdVel = rawVel

    # Set desired velocity, but apply acceleration limiting
    def SetDesiredVelocity(self, desired_angular_z_vel, desired_linear_x_vel):
        newVel = Twist()
        (newVel.angular.z, newVel.linear.x) = self.LimitAcceleration(desired_angular_z_vel, desired_linear_x_vel)
        self.cmdVel = newVel

    # Compute desired velocity using PID
    def SetVelocity_PID(self, angularError, linearError, angularPID = None, linearPID = None):
        
        ang_u = lin_u = 0
        if not angularPID is None:
            ang_u = angularPID.GetControl(angularError, rospy.Time.now())

        if not linearPID is None:
            lin_u = linearPID.GetControl(linearError, rospy.Time.now())

        self.SetDesiredVelocity(ang_u, lin_u)


    def GetNextTwist(self):
        return self.cmdVel
