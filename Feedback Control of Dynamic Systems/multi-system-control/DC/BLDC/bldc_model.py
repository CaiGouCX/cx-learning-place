import numpy as np

class BLDC_MODEL:
    """
    bldc_model.py
    Brushless DC Motor (BLDC) model simulation
    """

    def __init__(self):
        # 电机参数
        self.R = 1.2             # 每相电阻 (Ω)
        self.L = 0.003           # 每相电感 (H)，即 3 mH，典型值
        self.J = 0.0208          # 转动惯量 (kg·m²)
        self.B = 0.0005          # 粘性摩擦系数 (N·m·s)
        self.p = 4               # 极对数
        self.Ke = 0.15           # 反电动势系数 (V·s/rad)

        # 状态变量
        self.theta = 0.0          # 机械角度 (rad)
        self.omega = 0.0          # 角速度 (rad/s)
        self.theta_e = 0.0

        self.i_a = 0.0
        self.i_b = 0.0
        self.i_c = 0.0

        self.e_a = 0.0
        self.e_b = 0.0
        self.e_c = 0.0

        self.TL = 0.0             # 当前负载转矩


    def get_state(self):
        return {
            "omega": self.omega,
            "theta": self.theta,
            "i_a": self.i_a,
            "i_b": self.i_b,
            "i_c": self.i_c,
            "e_a": self.e_a,
            "e_b": self.e_b,
            "e_c": self.e_c,
            "TL": self.TL
        }

    # -------------------------
    # 反电动势建模
    # -------------------------
    def trapezoidal_emf(self, theta_e):
        def wave(theta):
            theta = theta % (2 * np.pi)
            if 0 <= theta < np.pi/6:
                return 6 * theta / np.pi
            elif np.pi/6 <= theta < 5*np.pi/6:
                return 1.0
            elif 5*np.pi/6 <= theta < 7*np.pi/6:
                return 1 - 6 * (theta - 5*np.pi/6) / np.pi
            elif 7*np.pi/6 <= theta < 11*np.pi/6:
                return -1.0
            else:
                return -1 + 6 * (theta - 11*np.pi/6) / np.pi

        E = self.Ke * self.omega
        self.e_a = E * wave(theta_e)
        self.e_b = E * wave(theta_e - 2*np.pi/3)
        self.e_c = E * wave(theta_e + 2*np.pi/3)

    def sinusoidal_emf(self, theta_e):
        E = self.Ke * self.omega
        self.e_a = E * np.sin(theta_e)
        self.e_b = E * np.sin(theta_e - 2*np.pi/3)
        self.e_c = E * np.sin(theta_e + 2*np.pi/3)

    def update_emf(self, theta_e, mode='hall'):
        if mode == 'hall':
            self.trapezoidal_emf(theta_e)
        elif mode in ['sinusoidal', 'foc']:
            self.sinusoidal_emf(theta_e)

    # -------------------------
    # 负载建模
    # -------------------------
    def load_torque(self, t, omega, mode='hall'):
        T_base = 0.02         # 基础负载 (Nm)
        k_fan = 5e-4          # 风阻系数
        A_disturb = 0.005     # 扰动幅值
        f_disturb = 10        # 扰动频率 (Hz)

        if mode == 'hall':
            return T_base
        elif mode == 'sinusoidal':
            return T_base + k_fan * omega**2
        elif mode == 'foc':
            return T_base + k_fan * omega**2 + A_disturb * np.sin(2 * np.pi * f_disturb * t)

    # -------------------------
    # 主状态更新函数
    # -------------------------
    def update(self, Va, Vb, Vc, dt, mode):

        self.update_emf(self.theta_e, mode)

        dia_dt = (Va - self.R * self.i_a - self.e_a) / self.L
        dib_dt = (Vb - self.R * self.i_b - self.e_b) / self.L
        dic_dt = (Vc - self.R * self.i_c - self.e_c) / self.L

        self.TL = self.load_torque(dt, self.omega, mode)
        if self.omega == 0:
            self.omega+=0.001

        domega_dt = ((self.e_a * self.i_a + self.e_b * self.i_b + self.e_c * self.i_c) / self.omega - self.TL - self.B * self.omega) / self.J
        

        # 欧拉积分
        self.i_a += dia_dt * dt
        self.i_b += dib_dt * dt
        self.i_c += dic_dt * dt
        self.omega += domega_dt * dt
        self.theta_e += self.omega * dt
        self.theta = self.theta_e % (2 * np.pi)  # 限制在 0~2π
        #print(self.theta_e)


        return self.get_state()
