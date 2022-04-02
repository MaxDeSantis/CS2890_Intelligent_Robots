import rospy

from geometry_msgs.msg import Twist
import robot_pid


class VelocityManager:

    def __init__(self, ang_acc, lin_acc):
        print ("Init velocity manager")
        
        

        # Acceleration Clamps
        self.max_ang_acc = abs(ang_acc)
        self.max_lin_acc = abs(lin_acc)

        # Measured Values
        self.angle_measured = -1
        self.dist_measured = -1

        # Actual values
        self.ang_vel_actual = 0
        self.lin_vel_actual = 0

        self._nextTwist = Twist()


    # Clamp desired velocity according to maximum acceleration
    def SetDesiredVelocity(self, desired_ang_z_vel, desired_lin_x_vel):
        #print("setting velocity")
        new_vel = Twist()

        # Clamp angular
        if desired_ang_z_vel - self.ang_vel_actual > self.max_ang_acc:
            # Positive, greater than maximum angular acc.
            new_vel.angular.z = self.ang_vel_actual + self.max_ang_acc

        elif desired_ang_z_vel - self.ang_vel_actual < -self.max_ang_acc:
            # Negative, less than minimum angular acc (negative max)
            new_vel.angular.z = self.ang_vel_actual - self.max_ang_acc

        else:
            # Within limits, go ahead
            new_vel.angular.z = desired_ang_z_vel

        # Clamp linear
        if desired_lin_x_vel - self.lin_vel_actual > self.max_lin_acc:
            new_vel.linear.x = self.lin_vel_actual + self.max_lin_acc

        elif desired_lin_x_vel - self.lin_vel_actual < -self.max_lin_acc:
            new_vel.linear.x = self.lin_vel_actual - self.max_lin_acc

        else:
            new_vel.linear.x = desired_lin_x_vel
    

        # Set next velocity to be actuated
        self._nextTwist.angular.z = new_vel.angular.z
        self._nextTwist.linear.x = new_vel.linear.x

        # Update memory of last actuated velocity
        self.ang_vel_actual = new_vel.angular.z
        self.lin_vel_actual = new_vel.linear.x
        

    # Returns final twist to SoccerBot for actuation in Run loop
    def GetNextTwist(self):
        #print("Returning next twist")
        return self._nextTwist
