import config


class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.last_error = 0

    def update(self, target, measured):
        error = target - measured
        d_error = error - self.last_error
        output = self.kp * error - self.kd * d_error
        self.last_error = error
        return output


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.last_error = 0.0
        self._has_last = False
        self._filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self._has_last = False
        self._filtered_derivative = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3

        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        raw_derivative = 0.0
        if self._has_last:
            raw_derivative = (error - self.last_error) / dt
        self.last_error = error
        self._has_last = True

        alpha = config.PID_DERIVATIVE_FILTER_ALPHA
        self._filtered_derivative += (raw_derivative - self._filtered_derivative) * alpha
        derivative = self._filtered_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        return output

    def sync_error(self, error: float):
        self.last_error = error
        self._has_last = True
        self._filtered_derivative = 0.0
