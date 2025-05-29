class PID:
    def __init__(self, kp, ki, kd, dt=0.1, output_limits=(0,100)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt                 = dt
        self.integral           = 0
        self.prev_error         = 0
        self.min_out, self.max_out = output_limits

    def reset(self):
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        if self.min_out is not None: output = max(self.min_out, output)
        if self.max_out is not None: output = min(self.max_out, output)
        self.prev_error = error
        return output
