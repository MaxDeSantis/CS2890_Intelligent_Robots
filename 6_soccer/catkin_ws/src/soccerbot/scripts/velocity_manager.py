from asyncio.windows_events import NULL
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
                                    self.parameterManager.THETA_MAX,
                                    self.parameterManager.THETA_MIN)
        
        self.cmdVel             = Twist()

    # Returns angular z and linear x clamped to ensure acceleration is within maximums.
    def LimitAcceleration(self, desired_angular_z_vel, desired_linear_x_vel):
        # Clamp angular
        if desired_angular_z_vel - self.gameState.soccerbot_ang_z > self.parameterManager.MAX_ANGULAR_ACCELERATION:
            # Positive, greater than maximum angular acc.
            limited_angular_z_vel = self.gameState.soccerbot_ang_z + self.max_ang_acc
        elif desired_angular_z_vel - self.gameState.soccerbot_ang_z < -self.parameterManager.MAX_ANGULAR_ACCELERATION:
            # Negative, less than minimum angular acc (negative max)
            limited_angular_z_vel = self.gameState.soccerbot_ang_z - self.parameterManager.MAX_ANGULAR_ACCELERATION
        else:
            # Within limits, go ahead
            limited_angular_z_vel = desired_angular_z_vel

        # Clamp linear
        if desired_linear_x_vel - self.gameState.soccerbot_lin_x > self.MAX_LINEAR_ACCELERATION:
            limited_linear_x_vel = self.gameState.soccerbot_lin_x + self.MAX_LINEAR_ACCELERATION
        elif desired_linear_x_vel - self.gameState.soccerbot_lin_x < -self.MAX_LINEAR_ACCELERATION:
            limited_linear_x_vel = self.gameState.soccerbot_lin_x - self.MAX_LINEAR_ACCELERATION
        else:
            limited_linear_x_vel = desired_linear_x_vel

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

        if not angularPID is None:
            ang_u = angularPID.GetControl(angularError, rospy.Time.now())

        if not linearPID is None:
            lin_u = linearPID.GetControl(linearError, rospy.Time.now())

        self.SetDesiredVelocity(ang_u, lin_u)


    def GetNextTwist(self):
        return self.cmdVel



    # def __init__(self, ang_acc, lin_acc):
    #     print ("Init velocity manager")
        
        

    #     # Acceleration Clamps
    #     self.max_ang_acc = abs(ang_acc)
    #     self.max_lin_acc = abs(lin_acc)

    #     # Measured Values
    #     self.angle_measured = -1
    #     self.dist_measured = -1

    #     # Actual values
    #     self.ang_vel_actual = 0
    #     self.lin_vel_actual = 0

    #     self._nextTwist = Twist()


    # # Clamp desired velocity according to maximum acceleration
    # def SetDesiredVelocity(self, desired_ang_z_vel, desired_lin_x_vel):
    #     #print("setting velocity")
    #     new_vel = Twist()

    #     # Clamp angular
    #     if desired_ang_z_vel - self.ang_vel_actual > self.max_ang_acc:
    #         # Positive, greater than maximum angular acc.
    #         new_vel.angular.z = self.ang_vel_actual + self.max_ang_acc

    #     elif desired_ang_z_vel - self.ang_vel_actual < -self.max_ang_acc:
    #         # Negative, less than minimum angular acc (negative max)
    #         new_vel.angular.z = self.ang_vel_actual - self.max_ang_acc

    #     else:
    #         # Within limits, go ahead
    #         new_vel.angular.z = desired_ang_z_vel

    #     # Clamp linear
    #     if desired_lin_x_vel - self.lin_vel_actual > self.max_lin_acc:
    #         new_vel.linear.x = self.lin_vel_actual + self.max_lin_acc

    #     elif desired_lin_x_vel - self.lin_vel_actual < -self.max_lin_acc:
    #         new_vel.linear.x = self.lin_vel_actual - self.max_lin_acc

    #     else:
    #         new_vel.linear.x = desired_lin_x_vel
    

    #     # Set next velocity to be actuated
    #     self._nextTwist.angular.z = new_vel.angular.z
    #     self._nextTwist.linear.x = new_vel.linear.x

    #     # Update memory of last actuated velocity
    #     self.ang_vel_actual = new_vel.angular.z
    #     self.lin_vel_actual = new_vel.linear.x
        

    # # Returns final twist to SoccerBot for actuation in Run loop
    # def GetNextTwist(self):
    #     #print("Returning next twist")
    #     return self._nextTwist
