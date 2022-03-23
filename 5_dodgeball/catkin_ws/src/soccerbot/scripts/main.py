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

print("test1")
if __name__ == "__main__":
    print("test2")
    rospy.init_node('robot_main')
    soccerBot = soccer_bot.SoccerBot()
    soccerBot.Run()
print("test3")
