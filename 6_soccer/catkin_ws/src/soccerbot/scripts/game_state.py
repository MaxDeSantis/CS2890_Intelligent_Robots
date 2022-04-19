import rospy
import enum
import math
import tf2_ros
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from soccerbot.msg import BallMeasured



class GameState:

    class BallGuess(enum.Enum):
        none    = 1,
        left    = 2,
        right   = 3,
        behind  = 4

    def __init__(self, parameter_manager):
        
        # General parameter access
        self.parameterManager = parameter_manager

        # Track opponent position
        self.opponent_goal_x = 0
        self.opponent_goal_y = 0
        self.opponent_x = 0
        self.opponent_y = 0
        self.opponent_ar_tag = 'ar_marker_' + str(int(self.parameterManager.OPPONENT_GOAL_ID))

        # Track ball position
        self.ball_x = 0
        self.ball_y = 0
        self.ball_bearing = -1
        self.ball_distance = -1
        self.ball_guess = self.BallGuess.behind
        self.theta_guess = None

        # Track self position
        self.own_goal_x = 0
        self.own_goal_y = 0
        self.soccerbot_x = 0
        self.soccerbot_y = 0
        self.soccerbot_theta = 0
        self.soccerbot_ar_tag        = 'ar_marker_' + str(int( not self.parameterManager.OPPONENT_GOAL_ID))

        # Track computed objective
        self.objective_x = 0
        self.objective_y = 0

        # Setup ROS subs and callbacks from object finder

        rospy.Subscriber('/odom', Odometry, self.HandleSoccerbotPose)
        rospy.Subscriber('/soccerbot/ball/pose', PoseStamped, self.HandleBallLocation)
        rospy.Subscriber('/soccerbot/opponent/pose', PoseStamped, self.HandleOpponentLocation)
        #rospy.Subscriber('/soccerbot/opponent/goal_pose', PoseStamped, self.HandleOpponentGoalLocation)
        #rospy.Subscriber('/soccerbot/goal_pose', PoseStamped, self.HandleSelfGoalLocation)
        rospy.Subscriber('/soccerbot/ball/measured', BallMeasured, self.HandleBallMeasurement)
        
        self.opponent_goal_pub = rospy.Publisher('/soccerbot/goals/opponent/pose', PoseStamped, queue_size=1)
        self.own_goal_pub = rospy.Publisher('/soccerbot/goals/self/pose', PoseStamped, queue_size=1)

        self.listener = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.listener)
        
        


    # Handle own pose in odom frame
    def HandleSoccerbotPose(self, msg):
        # Get current position
        self.soccerbot_x = msg.pose.pose.position.x
        self.soccerbot_y = msg.pose.pose.position.y

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (_, _, self.soccerbot_theta) = euler_from_quaternion(q)
        
        # Get current velocities
        self.soccerbot_lin_x = msg.twist.twist.linear.x
        self.soccerbot_ang_z = msg.twist.twist.angular.z

    # Get new ball bearing and distance to camera
    def HandleBallMeasurement(self, msg):
        self.ball_bearing = msg.bearing
        self.ball_distance = msg.distance
        
        #if ball_bearing > 320:
            #self.ball_guess = self.BallGuess.right
        #elif ball_bearing < 320:
            #self.ball_guess = self.BallGuess.left
        

    # Get new ball pose in odom frame
    def HandleBallLocation(self, msg):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y
        


    def HandleOpponentLocation(self, msg):
        print("thing")
    def HandleOpponentGoalLocation(self, msg):
        print("thing3")
    def HandleSelfGoalLocation(self, msg):
        print("thing2")
        
    def UpdateGoalTrackers(self):
        try:
            transform = self.listener.lookup_transform('odom', self.opponent_ar_tag, rospy.Time())
            self.opponent_goal_x = transform.transform.translation.x
            self.opponent_goal_y = transform.transform.translation.y
            opponentGoalPose = PoseStamped()
            opponentGoalPose.header.frame_id = "odom"
            opponentGoalPose.pose.position.x = self.opponent_goal_x
            opponentGoalPose.pose.position.y = self.opponent_goal_y
            self.opponent_goal_pub.publish(opponentGoalPose)
            
        except tf2_ros.LookupException:
            pass
        except tf2_ros.ConnectivityException:
            pass
        except tf2_ros.ExtrapolationException:
            pass
    
        try:
            transform = self.listener.lookup_transform('odom', self.soccerbot_ar_tag, rospy.Time())
            self.own_goal_x = transform.transform.translation.x
            self.own_goal_y = transform.transform.translation.y
            ownGoalPose = PoseStamped()
            ownGoalPose.header.frame_id = "odom"
            ownGoalPose.pose.position.x = self.own_goal_x
            ownGoalPose.pose.position.y = self.own_goal_y
            self.own_goal_pub.publish(ownGoalPose)
        except tf2_ros.LookupException:
            pass
        except tf2_ros.ConnectivityException:
            pass
        except tf2_ros.ExtrapolationException:
            pass
            
           
