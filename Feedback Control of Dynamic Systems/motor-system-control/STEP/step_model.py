import numpy as np
import matplotlib.pyplot as plt

class StepperMotor:
    def __init__(self, R=1.0, L=0.01, Ke=0.02, Kt=0.02,
                 J=0.001, B=0.0001, N_r=50, Ts=0.001):
        """
        N_r: 每机械转一圈电周期的数量（等于电机极对数 * 每圈步数）
        """
        self.R = R
        self.L = L
        self.Ke = Ke
        self.Kt = Kt
        self.J = J
        self.B = B
        self.N_r = N_r  # 每圈对应的电角度倍数
        self.Ts = Ts

        # 状态变量
        self.i_a = 0.0
        self.i_b = 0.0
        self.omega = 0.0
        self.theta = 0.0  # 机械角度（rad）

    def step(self, v_a, v_b):
        theta_e = self.N_r * self.theta  # 电角度

        # 电压-电流微分方程
        e_a = self.Ke * self.omega * np.sin(theta_e)
        e_b = self.Ke * self.omega * np.cos(theta_e)

        di_a = (v_a - self.R * self.i_a - e_a) / self.L
        di_b = (v_b - self.R * self.i_b - e_b) / self.L

        self.i_a += di_a * self.Ts
        self.i_b += di_b * self.Ts

        # 电磁转矩
        torque = self.Kt * (self.i_a * np.sin(theta_e) + self.i_b * np.cos(theta_e))

        # 机械运动方程
        domega = (torque - self.B * self.omega) / self.J
        self.omega += domega * self.Ts
        self.theta += self.omega * self.Ts
        self.theta %= 2 * np.pi  # 保持在 0~2π

        return self.i_a, self.i_b, self.omega, self.theta


