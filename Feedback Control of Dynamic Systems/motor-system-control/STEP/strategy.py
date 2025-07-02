import numpy as np
from pid import PID


# ======== 控制策略类 ========
class SineWaveController:
    """正弦波微步控制"""
    def __init__(self, motor, V=4.0, freq=100.0, direction=1):
        self.motor = motor
        self.V = V
        self.freq = freq
        self.direction = direction
        self.theta_e = 0.0

    def set_speed(self, freq):
        self.freq = freq

    def set_voltage(self, V):
        self.V = V

    def set_direction(self, direction):
        assert direction in [1, -1]
        self.direction = direction

    def update(self):
        self.theta_e += self.direction * 2 * np.pi * self.freq * self.motor.Ts
        v_a = self.V * np.sin(self.theta_e)
        v_b = self.V * np.cos(self.theta_e)
        return self.motor.step(v_a, v_b)

class FullStepController:
    """全步控制（单四相通电）"""
    def __init__(self, motor, step_rate_hz=100, V=4.0, direction=1):
        self.motor = motor
        self.step_rate_hz = step_rate_hz
        self.V = V
        self.direction = direction
        self.Ts = motor.Ts
        self.sequence = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        self.step_idx = 0
        self.timer = 0.0

    def update(self):
        self.timer += self.Ts
        if self.timer >= 1.0 / self.step_rate_hz:
            self.step_idx = (self.step_idx + self.direction) % len(self.sequence)
            self.timer = 0.0
        va = self.V * self.sequence[self.step_idx][0]
        vb = self.V * self.sequence[self.step_idx][1]
        return self.motor.step(va, vb)

class HalfStepController:
    """半步控制（交替通电）"""
    def __init__(self, motor, step_rate_hz=100, V=4.0, direction=1):
        self.motor = motor
        self.step_rate_hz = step_rate_hz
        self.V = V
        self.direction = direction
        self.Ts = motor.Ts
        self.sequence = [
            (1, 0), (1, 1), (0, 1), (-1, 1),
            (-1, 0), (-1, -1), (0, -1), (1, -1)
        ]
        self.step_idx = 0
        self.timer = 0.0

    def update(self):
        self.timer += self.Ts
        if self.timer >= 1.0 / self.step_rate_hz:
            self.step_idx = (self.step_idx + self.direction) % len(self.sequence)
            self.timer = 0.0
        va = self.V * self.sequence[self.step_idx][0]
        vb = self.V * self.sequence[self.step_idx][1]
        return self.motor.step(va, vb)

class PositionController:
    """改进：微步角度逼近目标，取消同步防止振荡"""
    def __init__(self, motor, V=4.0, step_angle=np.pi/100, direction=1):
        self.motor = motor
        self.V = V
        self.step_angle = step_angle
        self.direction = direction
        self.target = 0.0
        self.theta_e = motor.theta  # 初始角度

    def set_target(self, angle):
        self.target = angle

    def update(self):
        error = self.target - self.motor.theta
        
        # 动态调节步进速度（越近越慢）
        dynamic_step = self.step_angle * min(1.0, abs(error) / np.radians(5))

        # 更新控制角度（微步推进，不再同步回电机）
        if error > 0:
            self.theta_e += dynamic_step
        elif error < 0:
            self.theta_e -= dynamic_step

        # 驱动电压信号
        v_a = self.V * np.sin(self.theta_e * self.motor.N_r)
        v_b = self.V * np.cos(self.theta_e * self.motor.N_r)

        return self.motor.step(v_a, v_b)


class PIDSpeedController:
    def __init__(self, motor, setpoint_rads, V=4.0):
        self.motor = motor
        self.setpoint = setpoint_rads
        self.V = V
        self.Ts = motor.Ts
        self.theta_e = 0.0
        self.pid = PID(kp=1.5, ki=5.0, kd=0.01, dt=self.Ts, output_limits=(-200, 200))

    def update(self):
        # 反馈角速度取反，保证与setpoint同方向
        feedback = -self.motor.omega
        freq = self.pid.compute(self.setpoint, feedback)
        
        self.theta_e += 2 * np.pi * freq * self.Ts
        self.theta_e %= 2 * np.pi

        v_a = self.V * np.sin(self.theta_e)
        v_b = self.V * np.cos(self.theta_e)

        print(f"freq={freq:.3f}, theta_e={self.theta_e:.3f}, motor.omega={self.motor.omega:.3f}")
        return self.motor.step(v_a, v_b)



