import numpy as np
import matplotlib.pyplot as plt
from im_model import IMModel
from control_strategy import  PIDSpeedControl, FOCControl

def volt_fun(theta_e, voltage):
    """根据电角度生成三相正弦波电压"""
    va = voltage * np.sin(theta_e)
    vb = voltage * np.sin(theta_e - 2*np.pi/3)
    vc = voltage * np.sin(theta_e + 2*np.pi/3)
    return va, vb, vc

def main():
    dt = 0.001
    total_time = 0.5
    steps = int(total_time / dt)

    motor = IMModel()
    motor.set_load_torque(0.5)

    control_mode='foc'
    
    # 定义电机参数 (用于FOC控制)
    motor_params = {
        'p': motor.p,
        'Ls': motor.Ls,
        'Lr': motor.Lr,
        'B': motor.B
    }

    if control_mode == 'vf':
        print("vf")
    elif control_mode == 'pid':
        controller = PIDSpeedControl(setpoint=150, V_max=220, f_max=100,dt=0.001)
    elif control_mode == 'foc':
        controller = FOCControl(setpoint=150, motor_params=motor_params)
    else:
        raise ValueError("Unknown control mode")
    
    # 数据记录
    times, speeds, ids, iqs, teas = [], [], [], [], []
    VA, VB, VC = [], [], []

    theta_sync = 0.0  # 电角度积分
    freq_base = 50  # V/f 基础频率
    voltage_base = 220  # V/f 基础电压

    for i in range(steps):
        t = i * dt

        # 获取电机当前状态
        omega = motor.omega_r
        theta_r = motor.theta_r
        ids_val = motor.ids
        iqs_val = motor.iqs

        if control_mode == 'vf':
                # V/f 控制
            freq = min(freq_base, freq_base * t / 0.1)  # 前0.1秒匀速上升
            voltage = voltage_base * (freq / freq_base)
            theta_sync += 2 * np.pi * freq * dt  # 电角度积分

            va, vb, vc = volt_fun(theta_sync, voltage)

        elif control_mode == 'pid':

            (va, vb, vc), freq = controller.update( t, omega)
            
        elif control_mode == 'foc':

            if t < 0.28:
                    # V/f 控制
                freq = min(freq_base, freq_base * t / 0.1)  # 前0.1秒匀速上升
                voltage = voltage_base * (freq / freq_base)
                theta_sync += 2 * np.pi * freq * dt  # 电角度积分
                va, vb, vc = volt_fun(theta_sync, voltage)
            else:
                
                (va, vb, vc), freq = controller.update(theta_r, omega, ids_val, iqs_val,freq)

        motor.set_input_voltage_abc(va, vb, vc, freq)
        print(freq)
        state = motor.state_history[-1]
        times.append(t)
        speeds.append(state['omega_r'])
        ids.append(state['ids'])
        iqs.append(state['iqs'])
        teas.append(state['Te'])
        VA.append(va), VB.append(vb), VC.append(vc)

    # === 绘图 ===
    plt.figure()
    plt.plot(times, speeds, label='Speed ωr (rad/s)')
    plt.ylabel("Speed (rad/s)")
    plt.xlabel("Time (s)")
    plt.grid()
    plt.legend()

    plt.figure()
    plt.plot(times, ids, label="Ids")
    plt.plot(times, iqs, label="Iqs")
    plt.ylabel("Current (A)")
    plt.xlabel("Time (s)")
    plt.grid()
    plt.legend()

    plt.figure()
    plt.plot(times, VA, label='Va')
    plt.plot(times, VB, label='Vb')
    plt.plot(times, VC, label='Vc')
    plt.ylabel("Voltage (V)")
    plt.xlabel("Time (s)")
    plt.title("Three-phase voltages")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
