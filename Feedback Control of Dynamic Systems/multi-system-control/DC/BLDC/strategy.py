# strategy.py
import numpy as np

class Strategy:
    def __init__(self, mode='hall', max_voltage=24.0):
        self.mode = mode
        self.max_voltage = max_voltage


    def set_mode(self, mode):
        if mode not in ['hall', 'sinusoidal', 'foc']:
            raise ValueError("Unsupported mode")
        self.mode = mode

    def set_voltage(self, voltage):
        self.max_voltage = voltage

    def compute_voltage(self, theta_e, omega):
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

        elif self.mode in ['sinusoidal', 'foc']:
            # 三相正弦波驱动
            va = self.max_voltage * np.sin(theta_e)
            vb = self.max_voltage * np.sin(theta_e - 2*np.pi/3)
            vc = self.max_voltage * np.sin(theta_e - 4*np.pi/3)
            return va, vb, vc


'''

import numpy as np

class Strategy:
    def __init__(self, type='six_step', max_voltage=24.0, pole_pairs=4):
        self.type = type
        self.max_voltage = max_voltage
        self.pole_pairs = pole_pairs

    def get_voltage(self, theta_mech, t=0):
        theta_elec = (theta_mech * self.pole_pairs) % (2 * np.pi)

        if self.type == 'six_step':
            return self._six_step(theta_elec)
        elif self.type == 'sine':
            return self._sine_wave(theta_elec)
        elif self.type == 'foc_open_loop':
            return self._open_loop_foc(theta_elec)
        elif self.type == 'bemf_sensorless':
            return self._bemf_mock(t)
        else:
            raise ValueError(f"Unsupported strategy type: {self.type}")

    def _six_step(self, theta_e):
        sector = int(theta_e // (np.pi / 3))  # 6 sectors
        pattern = [
            (1, -1, 0), (0, -1, 1), (-1, 0, 1),
            (-1, 1, 0), (0, 1, -1), (1, 0, -1)
        ]
        va, vb, vc = pattern[sector]
        return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc

    def _sine_wave(self, theta_e):
        va = np.sin(theta_e)
        vb = np.sin(theta_e - 2 * np.pi / 3)
        vc = np.sin(theta_e - 4 * np.pi / 3)
        return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc

    def _open_loop_foc(self, theta_e):
        # Assume 90° current phase difference (Iq only)
        va = np.sin(theta_e)
        vb = np.sin(theta_e - 2 * np.pi / 3)
        vc = np.sin(theta_e - 4 * np.pi / 3)
        return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc

    def _bemf_mock(self, t):
        # Approximate sector switching every 5ms (mock sensorless zero crossing)
        sector = int((t % 0.03) // 0.005) % 6
        pattern = [
            (1, -1, 0), (0, -1, 1), (-1, 0, 1),
            (-1, 1, 0), (0, 1, -1), (1, 0, -1)
        ]
        va, vb, vc = pattern[sector]
        return self.max_voltage * va, self.max_voltage * vb, self.max_voltage * vc
'''