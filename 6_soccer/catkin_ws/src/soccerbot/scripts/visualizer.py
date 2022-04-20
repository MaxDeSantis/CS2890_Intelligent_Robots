# Visualize everything's position using an image stream


import game_state
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import roslib
import rospy
import numpy as np
import math

class SoccerVisualizer:

    def __init__(self, game_state):
        print("init visualizer")
        
        self.image_width = 100 # Number of cols
        self.image_height = 100 # Number of rows
        self.gameState = game_state
        
        self.bridge = CvBridge()
   
        self.image_pub = rospy.Publisher('/soccerbot/game_view', Image, queue_size=1)
        
        # Room is ~10 meters across. Convert from 100 px to 10 meters. Multiply by 10.
   
    def UpdateDisplay(self):
        
        blank = np.zeros((self.image_height, self.image_width, 3), np.uint8) # init black rectangle
        
        #blank[:, 0:self.image_width//2] = (255, 0, 0)
        #blank[:, self.image_width//2:self.image_width] = (0, 255, 0)
        
        # COLS ARE X, ROWS ARE Y
        ball_x = int(self.gameState.ball_x*10 + self.image_width/2)
        ball_y = int(-self.gameState.ball_y*10 + self.image_height/2)
        
        
        # This is all screwed up the way Numpy and opencv handle image definitions. Need to redo it.
        bot_x = int(self.gameState.soccerbot_x * 10 + self.image_width/2)
        bot_y = int(-self.gameState.soccerbot_y * 10 + self.image_height/2)
        
        blank[int(self.image_width/2), int(self.image_height/4)] = (0, 0, 255)
        
        blank[int(self.image_width/2),int(self.image_height/2)] = (255, 255, 255) # make center white
        
        blank[ball_y, ball_x] = (255, 255, 150) # ball position is blue
        
        blank[bot_y, bot_x] = (0, 255, 0) # robot position is green
        blank[pointer_y, pointer_x] = (155, 155, 0)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(blank, "bgr8"))
