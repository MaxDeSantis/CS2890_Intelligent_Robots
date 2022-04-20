import rospy
import math

import game_state
import parameter_manager

from geometry_msgs.msg import Twist

# Implementation references this https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

class PotentialManager:
    def __init__(self, parameter_manager, game_state):
        self.parameterManager = parameter_manager
        self.gameState = game_state
        self.obstacles = []
    
    def Distance(self, x1, y1, x2, y2):
        d1 = (x1+x2)**2
        d2 = (y1+y2)**2
        d = math.sqrt(d1 + d2)
        return d

    def GetAttractiveForce(self):

        dStar = self.parameterManager.COMBINED_ATTRACTION_DIST_CUTOFF
        zeta = self.parameterManager.ATTRACTIVE_ZETA
        
        total_dist = self.Distance(self.gameState.soccerbot_x, self.gameState.soccerbot_y,
                                    self.gameState.objective_x, self.gameState.objective_y)

        if total_dist <= dStar:
            # Close to goal, use normal approach
            f_x = zeta * (self.gameState.soccerbot_x - self.gameState.objective_x)
            f_y = zeta * (self.gameState.soccerbot_y - self.gameState.objective_y)
        else:
            # Further from goal
            f_x = dStar * zeta * (self.gameState.soccerbot_x - self.gameState.objective_x) / total_dist
            f_y = dStar * zeta * (self.gameState.soccerbot_y - self.gameState.objective_y) / total_dist
        
    
        return (f_x, f_y)
        
    def GetRepulsiveForce(self):
        
        # Iterate through all obstacles, which should have an X,Y
        #for o in self.obstacles:

        # Ball repulsive force
        b_x = self.gameState.ball_x
        b_y = self.gameState.ball_y
        s_x = self.gameState.soccerbot_x
        s_y = self.gameState.soccerbot_y
        d_x = b_x - s_x
        d_y = b_y - s_y

        Qstar   = self.parameterManager.REPULSIVE_CUTOFF
        eta     = self.parameterManager.REPULSIVE_ETA

        dist = self.Distance(b_x, b_y, s_x, s_y)

        if dist <= Qstar:
            # Generate repulsive force
            f_x = eta * (1/Qstar - 1/dist) * (1/dist)**2 * (d_x / dist)
            f_y = eta * (1/Qstar - 1/dist) * (1/dist)**2 * (d_y / dist)
        else:
            f_x = 0
            f_y = 0

        return (f_x, f_y)



    # Return potential at robot's position
    def GetLocalPotential(self):
        #print("potential")
        (fx, fy) = tuple(map(lambda i, j: i + j, self.GetAttractiveForce(), self.GetRepulsiveForce()))
        print("fx", fx, "fy", fy)
        return (-fx, -fy)
