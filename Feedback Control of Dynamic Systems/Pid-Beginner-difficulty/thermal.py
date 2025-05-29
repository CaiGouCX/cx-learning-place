class ThermalSystem:
    def __init__(self, ambient=25.0, C=10.0, loss=0.1):
        self.temperature = ambient
        self.ambient     = ambient
        self.C           = C
        self.loss        = loss

    def update(self, power, dt):
        dT = (power - self.loss*(self.temperature - self.ambient)) / self.C
        self.temperature += dT * dt
        return self.temperature
