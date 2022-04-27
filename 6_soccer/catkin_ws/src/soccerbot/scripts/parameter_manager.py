

class ParameterManager:

    def __init__(self):
        
        # Goal Information
        self.OPPONENT_GOAL_ID = 1
        self.MAX_SEARCH_ERROR_THETA = .35

        # Velocity management
        self.MAX_LINEAR_ACCELERATION = .03
        self.MAX_ANGULAR_ACCELERATION = .3
        self.MAX_ANGULAR_VELOCITY = 1.3
        self.MAX_LINEAR_VELOCITY = 1.0
        self.MAX_ANG_Z_ERROR_CUTOFF = 0.1

        # PID ------------------
        self.BEARING_KP = 0.0053
        self.BEARING_KI = 0
        self.BEARING_KD = 0.0016
        self.BEARING_MAX = self.MAX_ANGULAR_VELOCITY
        self.BEARING_MIN = -self.MAX_ANGULAR_VELOCITY

        self.RANGE_KP = 0
        self.RANGE_KI = 0
        self.RANGE_KD = 0
        self.RANGE_MAX = 0
        self.RANGE_MIN = 0

        self.THETA_KP = 1.3
        self.THETA_KI = 0
        self.THETA_KD = .3
        self.THETA_MAX = self.MAX_ANGULAR_VELOCITY
        self.THETA_MIN = -self.MAX_ANGULAR_VELOCITY
        # ----------------------

        # Potential Field ------
        self.COMBINED_ATTRACTION_DIST_CUTOFF    = 0.1
        self.ATTRACTIVE_ZETA                    = 3.8
        
        self.BALL_REPULSIVE_ETA                 = 0.7
        self.BALL_REPULSIVE_CUTOFF              = 0.56
        self.POTENTIAL_MAG_LOWER_CUTOFF         = 0.03
        
        self.OBSTACLE_MEMORY_TIME               = 20
        self.OBS_REPULSIVE_ETA                  = 1.5
        self.OBS_REPULSIVE_CUTOFF               = 1.0
        self.OBS_DIST                           = 0.2
        
        self.NUDGE_FORCE                        = 0.5

        # ----------------------

        # Stop state
        self.STOP_DURATION = 1.0

        # Recover state
        self.RECOVER_DURATION = 1.5
        self.RECOVER_LINEAR_X = -0.2

        # Search State
        self.SEARCH_ANG_Z_DEFAULT = 1.3
        
        # Line Up
        self.MAX_LINUP_BEARING_ERROR = 35
        
        # Approach
        self.OBJECTIVE_DIST_FROM_BALL = 1.5
        self.MAX_GOAL_ERROR = 0.4
        
        # Kick Line Up
        self.MAX_BALL_ERROR_KICK = 0.2

        # Kick
        self.KICK_LIN_X = 1.5
        self.KICK_DURATION = 1.8
