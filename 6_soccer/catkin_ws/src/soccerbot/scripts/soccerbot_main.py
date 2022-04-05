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
        initial_localize        = 1


    def __init__(self):

        self.positionManager = position_manager.PositionManager()
        self.actionManager = action_manager.ActionManager(self.positionManager)


        self.gameState = game_state.GameState()
        self.state = self.RobotState.stop



    def RunFSM(self):
        print("run fsm")
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            
            
            
            
            
            self.gameState.Update()

            rate.sleep()




if __name__ == "__main__":
    rospy.init_node('soccerbot_main')
    soccerBot = SoccerBot()
    soccerBot.RunFSM()
