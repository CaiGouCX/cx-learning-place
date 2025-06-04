import matplotlib.pyplot as plt
from pid import PID   # 暂时不启用 PID 控制器
from bldc_model import BLDC_MODEL
from strategy import Strategy
import numpy as np

def main():
    dt = 0.001          # 仿真时间步长
    total_time = 0.5    # 总仿真时长（秒）
    steps = int(total_time / dt)

    motor = BLDC_MODEL()

    control = Strategy()
    control.set_mode("hall")
    control.set_voltage(220)
    
    pid = PID(kp=5, ki=0.8, kd=0.05, dt=dt, output_limits=(0, 220))

    setpoint = 314.0    # 目标转速（rad/s）

    times, speeds, Ia,Ib,Ic = [],[], [],[],[]
    Ea,Eb,Ec = [],[],[]

    #init

    omega = motor.get_state().get("omega")
    theta = motor.get_state().get("theta")


    for i in range(steps):
        t = i * dt

        # 用 PID 根据当前速度计算驱动电压
        voltage_pid = pid.compute(setpoint, omega)   # PID 输出电压
        control.set_voltage(voltage_pid)     # 设置控制器输出电压

        va,vb,vc =  control.compute_voltage(theta, omega)
        #print(va,vb,vc)
        # 三相电压简化为相同输入
        state = motor.update(va, vb, vc, dt, mode='hall')
        theta = state.get("theta")
        omega = state.get("omega")
        #print(omega)
        # 记录
        times.append(t)
        speeds.append(omega)
        Ia.append(state.get("i_a"))
        Ib.append(state.get("i_b"))
        Ic.append(state.get("i_c"))
        print(va)

        Ea.append(va)
        Eb.append(vb)
        Ec.append(vc)

    # 绘图

    #三相电流
    plt.figure(figsize=(10, 5))
    plt.plot(times, Ia, label='i_a (A)')
    plt.plot(times, Ib, label='i_b (A)')
    plt.plot(times, Ic, label='i_c (A)')
    plt.ylabel('Current (A)')
    plt.xlabel('time:t')
    plt.legend()
    plt.grid(True)


    #三相电压切换图   
    plt.figure(figsize=(10, 5))
    plt.plot(times, Ea, label='v_a (v)')
    plt.plot(times, Eb, label='v_b (v)')
    plt.plot(times, Ec, label='v_c (v)')
    plt.ylabel('Current (v)')
    plt.xlabel('time:t')
    plt.legend()
    plt.grid(True)
    
    #转速
    plt.figure(figsize=(10, 5))
    plt.title("Motor Speed Response")
    plt.plot(times, speeds, label='w (rad/s)')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('w (rad/s)')
    plt.xlabel('time:t')
    plt.legend()
    plt.grid(True)


    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
