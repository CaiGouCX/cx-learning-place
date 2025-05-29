class ThermalSystem2ndOrder:

    def __init__(self):
        self.T1 = 25.0  # 中间层温度（热惯性）
        self.T2 = 25.0  # 输出层温度

        self.C1 = 1000    # 加热棒（金属）热容 [J/K]
        self.C2 = 4180    # 1kg 水的热容 [J/K]
        self.a1 = 100     # 加热棒 → 水的传热 [W/K]
        self.a2 = 1.0     # 水 → 环境的散热 [W/K]（假设有保温层）

    def update(self, power, dt):

        Q = power
        dT1 = dt * (Q - self.a1 * (self.T1 - 25)) / self.C1
        self.T1 += dT1
        dT2 = dt * (self.a1 * (self.T1 - self.T2) - self.a2 * (self.T2 - 25)) / self.C2
        self.T2 += dT2

        return self.T2

    @property
    def temperature(self):
        return self.T2
