class PIDController:
    """
    PID controller for steering based on lateral offset error.
    """
    def __init__(self, kp=0.01, ki=0.001, kd=0.001):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        """
        Compute PID output for given error.
        """
        # Proportional term
        p = self.kp * error
        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral
        # Derivative term
        d = self.kd * (error - self.prev_error) / (dt + 1e-6)
        self.prev_error = error
        # Control output
        control = p + i + d
        return 
