import rospy
import math
import random

import game_state
import parameter_manager

from geometry_msgs.msg import Twist

# Implementation references this https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

class PotentialManager:
    def __init__(self, parameter_manager, game_state):
        self.parameterManager = parameter_manager
        self.gameState = game_state
        self.obstacles = []
        
    def AddObstacle(self, obs_x, obs_y):
        time = rospy.Time.now() # obstacles last for 30 seconds
        obs = (obs_x, obs_y, time)
        self.obstacles.append(obs)
    
    def Distance(self, x1, y1, x2, y2):
        d1 = (x1-x2)**2
        d2 = (y1-y2)**2
        d = math.sqrt(d1 + d2)
        return d

    def GetAttractiveForce(self):

        dStar = self.parameterManager.COMBINED_ATTRACTION_DIST_CUTOFF
        zeta = self.parameterManager.ATTRACTIVE_ZETA
        
        total_dist = self.Distance(self.gameState.soccerbot_x, self.gameState.soccerbot_y,
                                    self.gameState.objective_x, self.gameState.objective_y)

        print('Dist to goal:', total_dist)

        if total_dist <= dStar:
            # Close to goal, use normal approach
            f_x = zeta * (self.gameState.objective_x - self.gameState.soccerbot_x)
            f_y = zeta * (self.gameState.objective_y - self.gameState.soccerbot_y)
        else:
            # Further from goal
            f_x = dStar * zeta * (self.gameState.objective_x - self.gameState.soccerbot_x) / total_dist
            f_y = dStar * zeta * (self.gameState.objective_y - self.gameState.soccerbot_y) / total_dist

        return (f_x, f_y)
        
    def GetRepulsiveForce(self, obs_x, obs_y, eta, cutoff):
        
        s_x = self.gameState.soccerbot_x
        s_y = self.gameState.soccerbot_y
        d_x = obs_x - s_x
        d_y = obs_y - s_y

        Qstar   = cutoff

        dist = self.Distance(obs_x, obs_y, s_x, s_y)
        print('Dist to ball:', dist)
        if dist <= Qstar:
            print("less than qstar")
            # Generate repulsive force
            f_x = eta * (1/Qstar - 1/dist) * (1/dist)**2 * (d_x / dist)
            f_y = eta * (1/Qstar - 1/dist) * (1/dist**2) * (d_y / dist)
        else:
            f_x = 0
            f_y = 0
        
        return (f_x, f_y)



    # Return potential at robot's position
    def GetLocalPotential(self):

        (fa_x, fa_y) = self.GetAttractiveForce()
        (fr_x, fr_y) = self.GetRepulsiveForce(self.gameState.ball_x, self.gameState.ball_y, self.parameterManager.BALL_REPULSIVE_ETA ,self.parameterManager.BALL_REPULSIVE_CUTOFF)
        
        for o in self.obstacles:
            if (rospy.Time.now() - o[2]).to_sec() > self.parameterManager.OBSTACLE_MEMORY_TIME:
                (fo_x, fo_y) = self.GetRepulsiveForce(o[0], o[1], self.parameterManager.OBS_REPULSIVE_ETA, self.parameterManager.OBS_REPULSIVE_CUTOFF)
                (fr_x, fr_y) = (fr_x + fo_x, fr_y + fo_y)
            else:
                self.obstacles.remove(o)
        
        (fx, fy) = (fa_x + fr_x, fa_y + fr_y)


        print('s_x', self.gameState.soccerbot_x, 's_y', self.gameState.soccerbot_y, 'g_x', self.gameState.objective_x, 'g_y', self.gameState.objective_y, 'b_x', self.gameState.ball_x, 'b_y', self.gameState.ball_y, 'Fa_x:', fa_x, 'Fa_y:', fa_y, 'Fr_x:', fr_x, 'Fr_y:', fr_y, 'Fx:', fx, 'Fy:', fy)
        

        f_mag = self.Distance(0, 0, fx, fy)
        goal_dist = self.Distance(self.gameState.soccerbot_x, self.gameState.soccerbot_y,
                                    self.gameState.objective_x, self.gameState.objective_y)
                                    
        if abs(goal_dist) <= self.parameterManager.MAX_GOAL_ERROR: #and f_mag <= self.parameterManager.POTENTIAL_MAG_LOWER_CUTOFF:
            acceptable = True
        else:
            acceptable = False
            
        if f_mag <= self.parameterManager.POTENTIAL_MAG_LOWER_CUTOFF and not acceptable:
            rand = random.uniform(0, 1)
            fx = fx + self.parameterManager.NUDGE_FORCE * rand * math.cos(self.gameState.soccerbot_theta)
            fy = fy + self.parameterManager.NUDGE_FORCE * rand * math.sin(self.gameState.soccerbot_theta)
            print('nudging!!!!')
        
        return (fx, fy, acceptable)
