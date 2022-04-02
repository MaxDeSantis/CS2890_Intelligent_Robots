# Main FSM and entry point

import rospy
from enum import Enum

class SoccerBot:

    class RobotState(Enum):
        stop                    = 0,
        initial_localize        = 1


    def __init__(self):
        print ("do something")




    def RunFSM(self):
        print("run fsm")





if __name__ == "__main__":
    rospy.init_node('soccerbot_main')
    soccerBot = SoccerBot()
    soccerBot.RunFSM()