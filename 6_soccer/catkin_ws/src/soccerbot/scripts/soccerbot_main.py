#!/usr/bin/env python3
# Main FSM and entry point

# General
import rospy
import enum

# SoccerBot management
import action_manager
import position_manager
import game_state


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
        
        # Behavior management
        self.positionManager = position_manager.PositionManager()
        self.actionManager = action_manager.ActionManager(self.positionManager)

        # State tracking
        self.gameState = game_state.GameState()
        self.state = self.RobotState.stop

        # ROS Hooks


    def StopBehavior(self):
        print("stop")

    def RecoverBehavior(self):
        print("recovering")

    def InitialLocalizeBehavior(self):
        print("start localizing")

    def SearchBehavior(self):
        print("search")


    
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


            rate.sleep()




if __name__ == "__main__":
    rospy.init_node('soccerbot_main')
    soccerBot = SoccerBot()
    soccerBot.RunFSM()
