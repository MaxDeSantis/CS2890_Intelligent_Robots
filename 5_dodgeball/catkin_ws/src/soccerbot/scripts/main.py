#!/usr/bin/env python3

from tracemalloc import stop
import roslib
import rospy
import soccer_bot



# ----------------------------------------------------------------------------------------
#
# Main Entry Point
#
# ----------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('robot_main')
    soccerBot = soccer_bot.SoccerBot()
    soccerBot.Run()
