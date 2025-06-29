import numpy as np

class IMModel:
    def __init__(self,
                 Rs=0.5, Rr=0.5,
                 Ls=0.01, Lr=0.01, Lm=0.008,
                 J=0.01, B=0.001,
                 p=2, Ts=0.001):
        self.Rs = Rs
        self.Rr = Rr
        self.Ls = Ls
        self.Lr = Lr
        self.Lm = Lm
        self.J = J
        self.B = B
        self.p = p
        self.Ts = Ts

        # 状态变量
        self.ids = 0.0
        self.iqs = 0.0
        self.idr = 0.0
        self.iqr = 0.0
        self.omega_r = 0.0
        self.theta_r = 0.0
        self.ws = 0.0
        self.Tl = 0.01

        self.state_history = []

    def set_stator_frequency(self, freq_hz):
        self.ws = 2 * np.pi * freq_hz

    def set_load_torque(self, Tl):
        self.Tl = Tl

    def abc_to_alpha_beta(self, a, b, c):
        alpha = (2 / 3) * (a - 0.5 * b - 0.5 * c)
        beta = (2 / 3) * ((np.sqrt(3) / 2) * (b - c))
        return alpha, beta

    def alpha_beta_to_dq(self, alpha, beta, theta):
        d = alpha * np.cos(theta) + beta * np.sin(theta)
        q = -alpha * np.sin(theta) + beta * np.cos(theta)
        return d, q

    def set_input_voltage_abc(self, va, vb, vc, freq):
        # 设置定子频率
        self.set_stator_frequency(freq)
        
        # 转换到αβ坐标系
        alpha, beta = self.abc_to_alpha_beta(va, vb, vc)
        
        # 转换到DQ坐标系 (使用电角度)
        theta_e = self.p * self.theta_r  # 机械角度转电角度
        vds, vqs = self.alpha_beta_to_dq(alpha, beta, theta_e)
        
        # 执行步进计算
        return self.step(vds, vqs)

    def step(self, vds, vqs):
        # 1. 计算电角度和转差
        theta_e = self.p * self.theta_r  # 机械角度转电角度
        slip = self.ws - self.p * self.omega_r  # 同步速度与转子电角速度差
        
        # 2. 计算转子磁链 (基于磁链方程)
        psi_dr = self.Lr * self.idr + self.Lm * self.ids
        psi_qr = self.Lr * self.iqr + self.Lm * self.iqs
        
        # 3. 计算电流微分 (基于电压方程)
        # 定子d轴: vds = Rs*ids + d(psi_ds)/dt - ws*psi_qs
        dids = (vds - self.Rs * self.ids + self.ws * self.Ls * self.iqs) / self.Ls
        
        # 定子q轴: vqs = Rs*iqs + d(psi_qs)/dt + ws*psi_ds
        diqs = (vqs - self.Rs * self.iqs - self.ws * self.Ls * self.ids) / self.Ls
        
        # 转子d轴: 0 = Rr*idr + d(psi_dr)/dt - slip*psi_qr
        didr = (-self.Rr * self.idr + slip * psi_qr) / self.Lr
        
        # 转子q轴: 0 = Rr*iqr + d(psi_qr)/dt + slip*psi_dr
        diqr = (-self.Rr * self.iqr - slip * psi_dr) / self.Lr
        
        # 4. 更新电流 (前向欧拉法)
        self.ids += dids * self.Ts
        self.iqs += diqs * self.Ts
        self.idr += didr * self.Ts
        self.iqr += diqr * self.Ts
        
        # 5. 计算电磁转矩 (基于转矩方程)
        Te = 1.5 * self.p * self.Lm * (self.iqs * self.idr - self.ids * self.iqr)
        
        # 6. 更新机械系统 (基于运动方程)
        # 机械方程: J*dω/dt = Te - Tl - B*ω
        domega = (Te - self.Tl - self.B * self.omega_r) / self.J
        self.omega_r += domega * self.Ts
        
        # 7. 更新转子位置 (积分角度)
        self.theta_r += self.omega_r * self.Ts
        self.theta_r %= 2 * np.pi  # 保持角度在[0, 2π]范围内
        
        # 8. 记录状态历史
        self.state_history.append({
            'ids': self.ids,
            'iqs': self.iqs,
            'idr': self.idr,
            'iqr': self.iqr,
            'omega_r': self.omega_r,
            'theta_r': self.theta_r,
            'Te': Te
        })
        
        return self.omega_r, Te