

import rospy
import enum
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class Robot:
    class RobotState(enum.Enum):
        moving_forward = 1
        moving_backwards = 2
        turning = 3

    def __init__(self):
        # Do something here
        self.current_state = self.RobotState.moving_forward

    def bumped(self, msg):
        # Got bumped, do something

    def run(self):
        rate = rospy.Rate(10)
        twist = Twist()
        
        while not rospy.is_shutdown():
            if self.current_state == self.RobotState.moving_forward:
                # Do something
            elif self.current_state == self.RobotState.moving_backwards:
                # Do something
            elif self.current_state == self.RobotState.turning:
                # Do something




rospy.init_node('prison_break')
robot = Robot()