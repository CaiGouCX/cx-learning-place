class FirstOrderThermalSystem:
    """
    一阶滞后热系统模拟（模拟锅炉）
    系统传递函数：
        G(s) = K / (tau*s + 1)
    其中：K 为系统增益，tau 为时间常数。
    """
    def __init__(self, K=0.5, tau=30.0):
        self.K = K
        self.tau = tau
        self.T = 25.0  # 初始温度（环境温度）

    @property
    def temperature(self):
        return self.T

    def update(self, power, dt):
        # 一阶惯性系统差分模型（Euler 法）
        dT = (-(self.T - 25.0) + self.K * power) / self.tau
        self.T += dT * dt
        return self.T
    
    