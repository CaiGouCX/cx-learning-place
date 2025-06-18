import matplotlib.pyplot as plt
import numpy as np
from pid import PID
from pmsm_model import PMSMModel

def volt_fun(direction,theta_e,voltage):
        # 根据方向调整相位
    if direction == 1:  # 正转
        va = voltage * np.sin(theta_e + np.pi)
        vb = voltage * np.sin(theta_e + np.pi - 2*np.pi/3)
        vc = voltage * np.sin(theta_e + np.pi + 2*np.pi/3)
    else:  # 反转
        # 反转时相位偏移180° (π弧度)
        va = voltage * np.sin(theta_e)
        vb = voltage * np.sin(theta_e - 2*np.pi/3)
        vc = voltage * np.sin(theta_e + 2*np.pi/3)
    return va,vb,vc

def main():
    dt = 0.001
    total_time = 0.5
    steps = int(total_time / dt)

    motor = PMSMModel()
    motor.set_direction(+1)
    motor.set_load_torque(0.01)

    pid = PID(kp=2, ki=0.5, kd=0.0, dt=dt, output_limits=(-220, 220))
    setpoint = 600.0
    switch_speed = 100.0  # 闭环切换临界转速

    motor.set_target_speed(setpoint)

    times, speeds, voltages, iqs, vffs, vpids = [], [], [], [], [], []
    VA , VB ,VC = [],[],[]

    for i in range(steps):
        t = i * dt
        omega = motor.get_speed()

        if t == 0.3:
            motor.set_target_speed(300)

        if omega < switch_speed:
            # 开环控制：估算电角度并施加正弦波三相电压（假设角频率 linearly 上升）
            theta_e = motor.theta_e
            Vmax = 220
            va,vb,vc = volt_fun(motor.direction,theta_e,Vmax)

        else:
            # 闭环 FOC 控制
            Vff = motor.get_Ke() * setpoint
            Vpid = pid.compute(setpoint, omega)
            voltage = np.clip(Vff + Vpid, 0, 220)
            last = motor.state_history[-1]
            theta_e = last['theta_e']
            va,vb,vc = volt_fun(motor.direction,theta_e,voltage)

            vffs.append(Vff)
            vpids.append(Vpid)
            voltages.append(voltage)

            
        VA.append(va),VB.append(vb),VC.append(vc)
        iq, omega_next = motor.set_input_voltage_abc(va, vb, vc)

        times.append(t)
        speeds.append(omega)
        iqs.append(iq)

    # 打印最后状态
    #last = motor.state_history[-1]
    #print("Final State:")
    #print(f"id: {last['id']:.4f}, iq: {last['iq']:.4f}, ωm: {last['omega_m']:.4f}, θe: {last['theta_e']:.4f}, Te: {last['Te']:.4f}")
    
    plt.subplot(2, 1, 1)
    plt.plot(times, speeds, label='Speed (rad/s)')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('ω (rad/s)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(times, iqs, label='iq (A)')
    plt.ylabel('iq')
    plt.legend()
    plt.grid(True)


    plt.figure(figsize=(10, 5))
    plt.plot(times, VA, label='v_a (V)')
    plt.plot(times, VB, label='v_b (V)')
    plt.plot(times, VC, label='v_c (V)')
    plt.title("Three-phase Voltages")
    plt.ylabel('Voltage (V)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
