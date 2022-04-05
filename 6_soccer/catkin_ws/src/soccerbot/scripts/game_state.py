

import rospy
import math
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped

class GameState:

    def __init__(self):
        
        # Track opponent position
        self.opponent_goal_x = 0
        self.opponent_goal_y = 0
        self.opponent_x = 0
        self.opponent_y = 0

        # Track ball position
        self.ball_x = 0
        self.ball_y = 0

        # Track self position
        self.own_goal_x = 0
        self.own_goal_y = 0
        self.soccerbot_x = 0
        self.soccerbot_y = 0
        self.soccerbot_theta = 0

        # Setup ROS subs and callbacks from object finder

        rospy.Subscriber('/odom', Odometry, self.HandlePose)
        rospy.Subscriber('/soccerbot/ball/pose', PoseStamped, self.HandleBallLocation)
        rospy.Subscriber('/soccerbot/opponent/pose', PoseStamped, self.HandleOpponentLocation)
        rospy.Subscriber('/soccerbot/opponent/goal_pose', PoseStamped, self.HandleOpponentGoalLocation)
        rospy.Subscriber('/soccerbot/goal_pose', PoseStamped, self.HandleSelfGoalLocation)

    def HandlePose(self, msg):
        self.soccerbot_x = msg.pose.pose.position.x
        self.soccerbot_y = msg.pose.pose.position.y

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (_, _, self.soccerbot_theta) = euler_from_quaternion(q)
        
        #self.soccerbot_ = msg.twist.twist.angular.z

    def HandleBallLocation(self, msg):

        # Check validity of reading and update
        if not math.isnan(msg.pose.position.x):
            print("do something")


    def HandleOpponentLocation(self, msg):
        print("thing")
    def HandleOpponentGoalLocation(self, msg):
        print("thing3")
    def HandleSelfGoalLocation(self, msg):
        print("thing2")

    