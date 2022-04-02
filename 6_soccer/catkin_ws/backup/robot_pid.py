
import rospy

class PID:
    def __init__(self, kp, ki, kd, max, min):
        # set gains from rosparams?
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = rospy.Time.now()
        self.max = max
        self.min = min

    def GetControl(self, setpoint, measured, current_time):
        # Compute control, return
        time_diff = (current_time - self.prev_time).to_sec()

        error = float(setpoint - measured)
        derivative = (error - self.prev_error) / time_diff
        self.integral += error
        
        #print("set: ", setpoint, " error: ", error)

        control = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Clamp output and handle integral windup
        if control > self.max:
            print("clamping to max: ", control)
            control = min(control, self.max)
            self.integral -= error
        elif control < self.min:
            print("clamping to min: ", control)
            control = max(control, self.min)
            self.integral -= error

        self.prev_error = error
        self.prev_time = current_time

        return control

