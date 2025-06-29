import numpy as np
from pid import PID

class PIDSpeedControl:
    """带PID速度闭环的V/f控制"""
    def __init__(self, setpoint, V_max=220, f_max=100, dt=0.001):
        """
        参数:
            setpoint: 目标转速 (rad/s)
            V_max: 最大电压 (V)
            f_max: 最大频率 (Hz)
            dt: 控制周期 (s)
        """
        self.setpoint = setpoint
        self.V_max = V_max
        self.f_max = f_max
        self.dt = dt
        
        # PID控制器用于调节电压幅值
        self.pid = PID(kp=8.0, ki=0.5, kd=0.01, dt=dt, output_limits=(0.1*V_max, V_max))
        
        # 状态变量
        self.theta_e = 0.0
        self.current_freq = 0.0
        self.ramp_completed = False
        self.ramp_time = 1 # 升频时间 (s)
        self.min_freq = 20.0   # 最小频率 (Hz)
        self.counter = 0
        
    def update(self, t, omega_r):
        """
        更新控制器
        参数:
            t: 当前时间 (s)
            omega_r: 当前转速 (rad/s)
        返回:
            (va, vb, vc): 三相电压 (V)
            freq: 输出频率 (Hz)
        """
        self.counter+=1
        # 1. 频率斜坡生成
        if omega_r > self.setpoint:
            print("movement",self.counter,omega_r,self.current_freq,t)
            self.ramp_completed = True
        if not self.ramp_completed:
            # 线性升频
            self.current_freq = min(self.f_max, self.min_freq + (self.f_max - self.min_freq) * t / self.ramp_time)
            if self.current_freq >= self.f_max:
                self.ramp_completed = True
        
        # 2. PID控制电压幅值
        voltage_base = self.V_max * (self.current_freq / self.f_max)  # 基本V/f曲线电压
        voltage_adjust = self.pid.compute(self.setpoint, omega_r)  # PID调整量
        
        # 组合电压 = 基础电压 + 调整量，但不超过最大电压
        voltage = min(voltage_base + voltage_adjust, self.V_max)
        
        # 3. 更新电角度
        self.theta_e = (self.theta_e + 2 * np.pi * self.current_freq * self.dt) % (2 * np.pi)
        
        # 4. 生成三相电压
        va = voltage * np.sin(self.theta_e)
        vb = voltage * np.sin(self.theta_e - 2 * np.pi / 3)
        vc = voltage * np.sin(self.theta_e + 2 * np.pi / 3)
        
        return (va, vb, vc), self.current_freq

    def reset(self):
        """重置控制器状态"""
        self.pid.reset()
        self.theta_e = 0.0
        self.current_freq = 0.0
        self.ramp_completed = False


class FOCControl:
    """矢量控制FOC,带速度环和电流环 (只支持正转)"""
    def __init__(self, setpoint, motor_params):
        """
        motor_params: 包含电机参数的字典
            p: 极对数
            Ld, Lq: dq轴电感
            lambda_pm: 永磁体磁链
        """
        self.setpoint = setpoint
        self.params = motor_params
       
        # 提取电机参数
        self.p = motor_params['p']          # 极对数
        self.Ld = motor_params['Ls']        # d轴电感
        self.Lq = motor_params['Lr']        # q轴电感
        self.lambda_pm = motor_params['B']  # 永磁体磁链

        # 计算d轴电流参考值 (固定值)
        self.id_ref = self.lambda_pm / ((self.Ld + self.Lq)/2 * 0.95)
        
        # 初始化PID控制器
        self.id_pid = PID(kp=15.0, ki=8, kd=0.01, dt=0.001, output_limits=(-220, 220))
        self.iq_pid = PID(kp=15.0, ki=8, kd=0.01, dt=0.001, output_limits=(-220, 220))


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
    
    def inv_clarke(self, v_alpha, v_beta):
        """基于您Clark变换的逆Clark变换"""
        # 从您的Clark变换推导:
        # i_alpha = ia
        # i_beta = (ia + 2 * ib) / np.sqrt(3)
        # 推导逆变换:
        va = v_alpha
        vb = (np.sqrt(3) * v_beta - v_alpha) / 2
        vc = -va - vb  # 因为 ia + ib + ic = 0
        return va, vb, vc

    def update_frequency(self,va,vb,vc, omega_e,freq):      
        """根据转速范围选择频率操作"""
        # 定义转速范围 (单位: rad/s)
        low_speed = self.setpoint-5
        mid_speed = self.setpoint
        high_speed = self.setpoint+5
        volt =  (abs(va)+abs(vb)+abs(vc))/3 
        if volt < 220:
            return freq
        if omega_e < low_speed  :
            # 低速区：升频加速
            
            return freq*1.1
        elif low_speed <= omega_e < high_speed :
            # 中速区：保持频率
            return freq
        else:
            # 高速区：降频减速
            return freq*0.98


    def update(self, theta_r, omega_r, ids, iqs,freq):
        # 计算电角度
        theta_e = (self.p * theta_r) % (2 * np.pi)
        omega_e = self.p * omega_r
        
        # 固定 iq_ref
        iq_ref = 150.0
        
        # 反电动势补偿
        vd_ff = -omega_e * self.Lq * iqs
        vq_ff = omega_e * (self.Ld * ids + self.lambda_pm)
        
        # 电流环控制
        vd = self.id_pid.compute(self.id_ref, ids) + vd_ff
        vq = self.iq_pid.compute(iq_ref, iqs) + vq_ff
        
        # 坐标变换
        v_alpha, v_beta = self.inv_park(vd, vq, theta_e)
        va, vb, vc = self.inv_clarke(v_alpha, v_beta)
        
        # 电压限幅
        va = max(min(va, 220), -220)
        vb = max(min(vb, 220), -220)
        vc = max(min(vc, 220), -220)
        
      
        return (va, vb, vc), self.update_frequency(omega_r ,va,vb,vc,freq)