#!/usr/bin/env python3
# Main FSM and entry point

# General
import rospy
import enum

# SoccerBot management
import velocity_manager
import parameter_manager
import game_state

from geometry_msgs.msg import Twist

# Overall Behavior:
# - FSM Manages State -> compute location we want to go.
# - Position manager implements pid controllers on theta and x, y
# - Velocity manager implements pid controller on output velocity
# - Object finder computes position of ball, opponent, and AR tag in odom frame.
# - Visualizer builds image to display positions of everything
# - GameState handles state callbacks to track everything's position

class SoccerBot:

    class RobotState(enum.Enum):
        stop                    = 0,
        recover                 = 1,
        initial_localize        = 2,
        search                  = 3


    def __init__(self):
        # State tracking
        self.state = self.RobotState.initial_localize
        self.waiting = False
        self.wait_time = [None, None]

        # Behavior management
        self.parameterManager = parameter_manager.ParameterManager()
        self.gameState = game_state.GameState(self.parameterManager)
        self.velocityManager = velocity_manager.VelocityManager(self.parameterManager, self.gameState)

        # ROS Hooks
        #rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.HandleBumped)
        self.motorPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    # -------------------------------------------------------------------------- Callbacks
    # Robot bumped something
    def HandleBumped(self, msg):
        self.state = self.RobotState.stop

    # -------------------------------------------------------------------------- Behaviors

    
    # Wait 'duration' seconds, returning whether or not still waiting. Meant for use in loops.
    def Waiting(self, duration):
        self.wait_time[1] = rospy.Time.now()

        if not self.waiting:
            self.wait_time[0] = rospy.Time.now()
            self.waiting = True
        else:
            if (self.wait_time[1] - self.wait_time[0]).to_sec() >= duration:
                self.waiting = False
        return self.waiting

    # Stop moving and wait specified duration. Transition to recover state afterwards.
    def StopBehavior(self):
        if not self.Waiting(self.parameterManager.STOP_DURATION):
            self.state = self.RobotState.recover
        self.velocityManager.SetRawVelocity(0, 0) # Stop completely

    # Backup slightly then enter search mode.
    def RecoverBehavior(self):
        if not self.Waiting(self.parameterManager.RECOVER_DURATION):
            self.state = self.RobotState.search
        self.velocityManager.SetRawVelocity(0, self.parameterManager.RECOVER_LINEAR_X) # Backwards slowly

    # Robot starts facing opponent goal.
    def InitialLocalizeBehavior(self):
        print("start localizing")

        if self.gameState.opponent_goal_x == 0 or self.gameState.opponent_goal_y == 0:
            print("Not found goal yet")
        else:
            print("Found goal")
            # Once it finds goal: enter search state - need to find ball
            self.state = self.RobotState.search
        
        self.velocityManager.SetRawVelocity(0, 0)

    # Rotate until the ball is found. Begins by rotating to initial estimate.
    def SearchBehavior(self):
        print("search")

        if self.gameState.ball_guess == self.gameState.BallGuess.left:
            print("left")
            
        elif self.gameState.ball_guess == self.gameState.BallGuess.right:
        
            print("right")
        
        elif self.gameState.ball_guess == self.gameState.BallGuess.behind:
            print("behind")
            # Turn around quick!
            self.velocityManager.SetVelocity_PID(self.gameState.theta_guess - self.gameState.soccerbot_theta,
                                                0, self.velocityManager.thetaPID, None)

        else:
            print("dont know")


    def RunFSM(self):
        print("run fsm")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            # AUTONOMOUS BEHAVIOR STATE MACHINE -----------------------
            if self.state == self.RobotState.stop:
                self.StopBehavior()
            elif self.state == self.RobotState.recover:
                self.RecoverBehavior()
            elif self.state == self.RobotState.initial_localize:
                self.InitialLocalizeBehavior()
            elif self.state == self.RobotState.search:
                self.SearchBehavior()
            else:
                print("UNDEFINED STATE")
            
            # ---------------------------------------------------------

            nextTwist = Twist()
            

            nextTwist = self.velocityManager.GetNextTwist()
            print(nextTwist)
            #self.motorPub.publish(nextTwist)
            
            self.gameState.UpdateGoalTrackers()


            rate.sleep()




if __name__ == "__main__":
    rospy.init_node('soccerbot_main')
    soccerBot = SoccerBot()
    soccerBot.RunFSM()
