import numpy as np

class PMSMModel:
    def __init__(self, Rs=0.5, Ld=0.004, Lq=0.001, J=0.01, B=0.0001,
                 psi_m=0.025, p=4, Ts=0.001):
        self.Rs = Rs
        self.Ld = Ld
        self.Lq = Lq
        self.J = J
        self.B = B
        self.psi_m = psi_m
        self.p = p  # 极对数
        self.Ts = Ts  # 仿真步长

        # 新增减速控制参数
        self.brake_gain = 1.2  # 减速增益因子
        self.min_iq = -0.5    # q轴电流下限（制动电流）
        self.current_kp = 1.685  # 电流控制增益（新添加）\
        self.target_speed = 0

        # 状态量
        self.id = 0.0
        self.iq = 0.0
        self.omega_m = 0.0
        self.theta_e = 0.0

        self.Tl = 0.0  # 机械负载
        self.direction = 1  # 方向（+1/-1）

        # 状态记录
        self.state_history = []

    def set_load_torque(self, Tl):
        self.Tl = Tl

    def set_direction(self, direction):
        self.direction = 1 if direction >= 0 else -1

    def get_speed(self):
        return self.omega_m

    def get_Ke(self):
        return 1.5 * self.p * self.psi_m


    def set_target_speed(self, target_speed):
        self.target_speed = target_speed
        self.sm_sign = True


    def abc_to_alpha_beta(self, a, b, c):
        alpha = (2/3) * (a - 0.5*b - 0.5*c)
        beta  = (2/3) * ((np.sqrt(3)/2)*(b - c))
        return alpha, beta

    def alpha_beta_to_dq(self, alpha, beta, theta):
        d =  alpha * np.cos(theta) + beta * np.sin(theta)
        q = -alpha * np.sin(theta) + beta * np.cos(theta)
        return d, q

    def dq_to_alpha_beta(self, d, q, theta):
        alpha = d * np.cos(theta) - q * np.sin(theta)
        beta  = d * np.sin(theta) + q * np.cos(theta)
        return alpha, beta

    def alpha_beta_to_abc(self, alpha, beta):
        a = alpha
        b = -0.5 * alpha + (np.sqrt(3)/2) * beta
        c = -0.5 * alpha - (np.sqrt(3)/2) * beta
        return a, b, c

    def set_input_voltage_abc(self, va, vb, vc):
        alpha, beta = self.abc_to_alpha_beta(va, vb, vc)
        ud, uq = self.alpha_beta_to_dq(alpha, beta, self.theta_e)
        return self.step(ud, uq)

    def step(self, ud, uq):

        # 在电气模型计算前添加减速控制逻辑
        if self.omega_m > self.target_speed:
            '''# 1. 计算目标制动电流
            speed_error = self.target_speed - self.omega_m
            iq_target = self.brake_gain * speed_error
            
            # 2. 限制制动电流范围
            iq_target = max(iq_target, self.min_iq)
            
            # 3. 计算电流误差
            iq_error = iq_target - self.iq
            
            # 4. 调整uq
            uq_adjustment = self.current_kp * iq_error
            uq += uq_adjustment'''
            uq = 0
            self.set_load_torque(100)#增加负载自然下降
        else:
            self.set_load_torque(0.01)

       

        did = (ud - self.Rs * self.id + self.Lq * self.omega_m * self.iq) / self.Ld
        diq = (uq - self.Rs * self.iq - self.Ld * self.omega_m * self.id - self.omega_m * self.psi_m) / self.Lq
        self.id += did * self.Ts
        self.iq += diq * self.Ts

        Te = 1.5 * self.p * (self.psi_m * self.iq + (self.Ld - self.Lq) * self.id * self.iq)
        domega = (Te - self.Tl - self.B * self.omega_m) / self.J
        self.omega_m += domega * self.Ts
        self.theta_e += self.p * self.omega_m * self.Ts
        self.theta_e = self.theta_e % (2 * np.pi)

        # 记录当前状态
        self.state_history.append({
            'id': self.id,
            'iq': self.iq,
            'omega_m': self.omega_m,
            'theta_e': self.theta_e,
            'Te': Te,
            'ud': ud,
            'uq': uq
        })

        return self.iq, self.omega_m
