from pid import PID
import numpy as np

class FOC:
    def __init__(self, max_voltage=220.0, pole_pairs=4, dt=1e-3):
        self.max_voltage = max_voltage
        self.p = pole_pairs
        self.dt = dt
        self.id_ref = 0.0
        self.iq_ref = 0.0
        self.pid_id = PID(kp=4.0, ki=0.5, kd=0.0, dt=dt, output_limits=(-max_voltage, max_voltage))
        self.pid_iq = PID(kp=4.0, ki=0.5, kd=0.0, dt=dt, output_limits=(-max_voltage, max_voltage))

    def set_voltage(self, voltage):
        self.max_voltage = voltage
        self.pid_id.set_max(voltage)
        self.pid_iq.set_max(voltage)

    def clarke(self, ia, ib, ic):#a相不动，bc合成，ic利用ia+ic+ic=0等价代换
        i_alpha = ia
        i_beta = (ia + 2 * ib) / np.sqrt(3)
        return i_alpha, i_beta

    def park(self, i_alpha, i_beta, theta_e):#旋转矩阵
        sin_theta = np.sin(theta_e)
        cos_theta = np.cos(theta_e)
        id =  i_alpha * cos_theta + i_beta * sin_theta
        iq = -i_alpha * sin_theta + i_beta * cos_theta
        return id, iq

    def inv_park(self, vd, vq, theta_e):
        sin_theta = np.sin(theta_e)
        cos_theta = np.cos(theta_e)
        v_alpha = vd * cos_theta - vq * sin_theta
        v_beta  = vd * sin_theta + vq * cos_theta
        return v_alpha, v_beta

    def svpwm(self, v_alpha, v_beta):
        #v_alpha, v_beta转三相电压
        v_a = v_alpha
        v_b = -0.5 * v_alpha + (np.sqrt(3)/2) * v_beta
        v_c = -0.5 * v_alpha - (np.sqrt(3)/2) * v_beta
        
        norm = max(abs(v_a), abs(v_b), abs(v_c))
        if norm < 1e-3:
            norm = 1.0  # 避免除零
        scale = self.max_voltage / norm

        return v_a * scale, v_b * scale, v_c * scale

    def update(self, ia, ib, ic, theta_e, omega, omega_ref,direction):
        # 1. 方向相关处理 - 使用统一的处理方式
        if direction == 1:
            # 反转时电角度偏移180°
            theta_e_direct = theta_e + np.pi
            # 速度参考值应为负值
            omega_ref_direct = -abs(omega_ref)
            # 实际速度也应取负值
            omega_direct = -abs(omega)
            # 反转时需要反转速度环输出
            speed_sign = -1
        else:
            theta_e_direct = theta_e
            omega_ref_direct = abs(omega_ref)
            omega_direct = abs(omega)
            speed_sign = 1

        # 2. 速度环控制 - 使用处理后的速度值
        # 添加方向符号控制
        self.iq_ref = speed_sign * self.pid_iq.compute(omega_ref_direct, omega_direct)
        
        # 3. 坐标变换
        i_alpha, i_beta = self.clarke(ia, ib, ic)
        id_val, iq_val = self.park(i_alpha, i_beta, theta_e_direct)  # 使用调整后的电角度
        
        # 4. 电流环控制
        vd = self.pid_id.compute(self.id_ref, id_val)
        vq = self.pid_iq.compute(self.iq_ref, iq_val)
        
        # 5. 逆变换
        v_alpha, v_beta = self.inv_park(vd, vq, theta_e_direct)  # 使用调整后的电角度
        
        # 6. SVPWM生成
        return self.svpwm(v_alpha, v_beta)

