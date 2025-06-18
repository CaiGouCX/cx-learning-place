# strategy.py
import numpy as np
from foc_controller import FOC
class Strategy:
    def __init__(self, mode='hall', max_voltage=24.0, pole_pairs=4, dt=0.001):
        self.mode = mode
        self.max_voltage = max_voltage
        self.foc = FOC(max_voltage=max_voltage, pole_pairs=pole_pairs, dt=dt)
        self.direction = 1  # 默认正转

    def set_direction(self, direction):
        self.direction = 1 if direction >= 0 else -1
           
    def set_mode(self, mode):
        if mode not in ['hall', 'sinusoidal', 'foc']:
            raise ValueError("Unsupported mode")
        self.mode = mode

    def set_voltage(self, voltage,mode='hall'):

        if mode == 'hall':
            self.max_voltage = voltage
        elif mode == 'foc': 
            self.foc.set_voltage(voltage)  

    def compute_voltage(self, theta_e, omega, omega_target=0, ia=0, ib=0, ic=0):
        
        """
        根据当前控制策略计算三相电压。
        参数：
            theta_e - 电角度（rad）
            omega   - 电机转速
        返回：
            Va, Vb, Vc
        """
        if theta_e > (2*np.pi):
            print("角度不对")
        if self.mode == 'hall':
            # 计算扇区 (0-5)
            sector = int((theta_e % (2*np.pi)) // (np.pi/3))
            
            # 根据方向选择模式向量
            if self.direction == 1:  # 正转
                pattern = [
                    (1, 0, 0),   # 扇区0: A+
                    (0, 1, -1),  # 扇区1: B+ C-
                    (-1, 1, -1), # 扇区2: A- B+ C-
                    (-1, 0, 0),  # 扇区3: A-
                    (0, -1, 1),  # 扇区4: B- C+
                    (1, -1, 1)   # 扇区5: A+ B- C+
                ]
                selected_sector = sector
            else:  # 反转
                pattern = [
                    (1, -1, 1),   # 扇区0: A+ B- C+ (对应正转扇区5)
                    (0, -1, 1),   # 扇区1: B- C+ (对应正转扇区4)
                    (-1, 0, 0),   # 扇区2: A- (对应正转扇区3)
                    (-1, 1, -1),  # 扇区3: A- B+ C- (对应正转扇区2)
                    (0, 1, -1),   # 扇区4: B+ C- (对应正转扇区1)
                    (1, 0, 0)     # 扇区5: A+ (对应正转扇区0)
                ]
                # 扇区映射: 反转扇区 = (5 - 正转扇区) % 6
                selected_sector = (5 - sector) % 6
            
            va, vb, vc = pattern[selected_sector]
            
            return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc

        elif self.mode == 'sinusoidal':
            # 三相正弦波驱动
            if self.direction == 1:  # 正转
                # 反转时相位偏移180° (π弧度)
                va = self.max_voltage * np.sin(theta_e)
                vb = self.max_voltage * np.sin(theta_e - 2*np.pi/3)
                vc = self.max_voltage * np.sin(theta_e - 4*np.pi/3)

            else:  # 反转

                va = self.max_voltage * np.sin(theta_e + np.pi)
                vb = self.max_voltage * np.sin(theta_e + np.pi - 2*np.pi/3)
                vc = self.max_voltage * np.sin(theta_e + np.pi - 4*np.pi/3)
            return va, vb, vc

        elif self.mode == 'foc':
            
            return self.foc.update(ia, ib, ic, theta_e, omega, omega_target,self.direction)
