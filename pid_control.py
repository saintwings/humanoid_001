import time

class PID_Control():
    def __init__(self, k_p, k_i, k_d, dt):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.dt = dt

        self.previous_error = 0
        self.integral = 0
    
    def update(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral = self.integral + error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = (self.k_p * error) + (self.k_i * self.integral) + (self.k_d * derivative)
        self.previous_error = error
        #time.sleep(self.dt)

        return output

    def reset(self):
        self.previous_error = 0
        self.integral = 0
