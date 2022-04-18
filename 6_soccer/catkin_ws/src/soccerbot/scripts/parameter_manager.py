

class ParameterManager:

    def __init__(self):
        
        # Goal Information
        self.OPPONENT_GOAL_ID = 0

        # Velocity management
        self.MAX_LINEAR_ACCELERATION = .1
        self.MAX_ANGULAR_ACCELERATION = .1

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

        self.THETA_KP = 0
        self.THETA_KI = 0
        self.THETA_KD = 0
        self.THETA_MAX = 0
        self.THETA_MIN = 0

        # Stop state
        self.STOP_DURATION = 0.5

        # Recover state
        self.RECOVER_DURATION = 1
        self.RECOVER_LINEAR_X = -0.2

        # Initial localize state

        # Search State
        self.SEARCH_ANG_Z_DEFAULT = 1
