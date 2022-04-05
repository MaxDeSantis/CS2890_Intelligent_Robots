

# Receive positional setpoints, control robot to reach that position.
# - Generate velocity setpoints -> send to velocity controller.

class PositionManager:
    def __init__(self):
        print("init position manager")