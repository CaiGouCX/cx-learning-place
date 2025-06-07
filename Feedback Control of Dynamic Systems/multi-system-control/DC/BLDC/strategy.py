# strategy.py
import numpy as np
from foc_controller import FOC
class Strategy:
    def __init__(self, mode='hall', max_voltage=24.0, pole_pairs=4, dt=0.001):
        self.mode = mode
        self.max_voltage = max_voltage
        self.foc = FOC(max_voltage=max_voltage, pole_pairs=pole_pairs, dt=dt)
        self.direction = 1  # 默认正转

    def set_direction(self, direction='forward'):
        if direction == 'forward':
            self.direction = 1
        elif direction == 'reverse':
            self.direction = -1
        else:
            raise ValueError("Invalid direction")    
           
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
            # 简化的六步换相电压控制（理想逻辑：0或最大值）
            sector = int((theta_e % (2*np.pi)) // (np.pi/3))  # 6个扇区
            pattern = [
                (1, 0, 0), (0, 1, -1), (-1, 1, -1),
                (-1, 0, 0), (0, -1, 1), (1, -1, 1)
            ]

            va, vb, vc = pattern[sector]
            return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc

        elif self.mode == 'sinusoidal':
            # 三相正弦波驱动
            va = self.max_voltage * np.sin(theta_e)
            vb = self.max_voltage * np.sin(theta_e - 2*np.pi/3)
            vc = self.max_voltage * np.sin(theta_e - 4*np.pi/3)
            return va, vb, vc

        elif self.mode == 'foc':
            
            return self.foc.update(ia, ib, ic, theta_e, omega, omega_target,self.direction)
