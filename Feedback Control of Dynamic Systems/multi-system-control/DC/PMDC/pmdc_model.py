class PMDC_MODEL:

# pmdc.py
# Permanent Magnet DC Motor (PMDC) model simulation

    def __init__(self, Ra=0.85, La=0.0023, K=0.042, J=0.00015, B=0.00018):

        # 电机参数
        self.Ra = Ra  # 电枢电阻 (Ω)
        self.La = La  # 电枢电感 (H)
        self.K = K    # 电机常数 (Nm/A = Vs/rad)
        self.J = J    # 转动惯量 (kg*m^2)
        self.B = B    # 粘性摩擦系数 (Nms)

        # 初始状态
        self.i_a = 0.0     # 电枢电流 (A)
        self.omega = 0.0   # 电机角速度 (rad/s)

    def reset(self):
        self.i_a = 0.0
        self.omega = 0.0

    def get_speed(self):
        return self.omega

    def get_Ke(self):
        return self.K    

    def update(self, Va, dt):
        # 微分方程求解（欧拉法）
        di_dt = (Va - self.Ra * self.i_a - self.K * self.omega) / self.La
        domega_dt = (self.K * self.i_a - self.B * self.omega) / self.J

        self.i_a += di_dt * dt
        self.omega += domega_dt * dt

        return self.i_a, self.omega

