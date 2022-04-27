

class ParameterManager:

    def __init__(self):
        
        # Goal Information
        self.OPPONENT_GOAL_ID = 1
        self.MAX_SEARCH_ERROR_THETA = .35

        # Velocity management
        self.MAX_LINEAR_ACCELERATION = .2
        self.MAX_ANGULAR_ACCELERATION = .3
        self.MAX_ANGULAR_VELOCITY = 2.0
        self.MAX_LINEAR_VELOCITY = 1.7
        self.MAX_ANG_Z_ERROR_CUTOFF = 0.1

        # PID ------------------
        self.BEARING_KP = 0.0059
        self.BEARING_KI = 0
        self.BEARING_KD = 0.0027
        self.BEARING_MAX = self.MAX_ANGULAR_VELOCITY
        self.BEARING_MIN = -self.MAX_ANGULAR_VELOCITY

        self.THETA_KP = 1.7
        self.THETA_KI = 0
        self.THETA_KD = .5
        self.THETA_MAX = self.MAX_ANGULAR_VELOCITY
        self.THETA_MIN = -self.MAX_ANGULAR_VELOCITY
        # ----------------------

        # Potential Field ------
        self.COMBINED_ATTRACTION_DIST_CUTOFF    = 0.1
        self.ATTRACTIVE_ZETA                    = 4.2
        
        self.BALL_REPULSIVE_ETA                 = 0.8
        self.BALL_REPULSIVE_CUTOFF              = 1.0
        self.POTENTIAL_MAG_LOWER_CUTOFF         = 0.03
        
        self.OBSTACLE_MEMORY_TIME               = 20
        self.OBS_REPULSIVE_ETA                  = 1.5
        self.OBS_REPULSIVE_CUTOFF               = 1.0
        self.OBS_DIST                           = 0.2
        
        self.NUDGE_FORCE                        = 1.7

        # ----------------------
        
        # State Behaviors ------

        # Stop state
        self.STOP_DURATION = 1.0

        # Recover state
        self.RECOVER_DURATION = 1.5
        self.RECOVER_LINEAR_X = -0.2

        # Search State
        self.SEARCH_ANG_Z_DEFAULT = 1.6
        
        # Line Up
        self.MAX_LINUP_BEARING_ERROR = 45
        
        # Approach
        self.OBJECTIVE_DIST_FROM_BALL = 1.0
        self.MAX_GOAL_ERROR = 0.3
        
        # Kick Line Up
        self.MAX_BALL_ERROR_KICK = 0.2

        # Kick
        self.KICK_LIN_X = 1.5
        self.KICK_DURATION = 1.8
