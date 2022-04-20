

class ParameterManager:

    def __init__(self):
        
        # Goal Information
        self.OPPONENT_GOAL_ID = 0
        self.MAX_SEARCH_ERROR_THETA = .2

        # Velocity management
        self.MAX_LINEAR_ACCELERATION = .1
        self.MAX_ANGULAR_ACCELERATION = .3
        self.MAX_ANGULAR_VELOCITY = 1.5
        self.MAX_LINEAR_VELOCITY = 1.0

        # PID ------------------
        self.BEARING_KP = 0
        self.BEARING_KI = 0
        self.BEARING_KD = 0
        self.BEARING_MAX = 0
        self.BEARING_MIN = 0

        self.RANGE_KP = 0
        self.RANGE_KI = 0
        self.RANGE_KD = 0
        self.RANGE_MAX = 0
        self.RANGE_MIN = 0

        self.THETA_KP = 1.3
        self.THETA_KI = 0
        self.THETA_KD = .1
        self.THETA_MAX = 0
        self.THETA_MIN = 0
        # ----------------------

        # Potential Field ------
        self.POTENTIAL_BALL_SIGMA               = 1.0
        self.POTENTIAL_OPPONENT_SIGMA           = 1.0
        self.POTENTIAL_OBSTACLE_SIGMA           = 1.0
        self.COMBINED_ATTRACTION_DIST_CUTOFF    = 1.0
        self.ATTRACTIVE_ZETA                    = 1.5
        self.REPULSIVE_ETA                      = 1.0
        self.REPULSIVE_CUTOFF                   = 0.5

        # ----------------------

        # Stop state
        self.STOP_DURATION = 0.5

        # Recover state
        self.RECOVER_DURATION = 1
        self.RECOVER_LINEAR_X = -0.2

        # Initial localize state

        # Search State
        self.SEARCH_ANG_Z_DEFAULT = 1

        # Approach
        self.OBJECTIVE_DIST_FROM_BALL = 1
