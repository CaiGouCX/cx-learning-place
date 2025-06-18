import matplotlib.pyplot as plt
from pid import PID
from pmdc_model import PMDC_MODEL
import numpy as np

def main():
    dt = 0.001  # 仿真时间步长
    total_time = 0.5  # 总仿真时长（秒）
    steps = int(total_time / dt)

    # 初始化系统和 PID 控制器
    motor = PMDC_MODEL()
    pid = PID(kp=0.75, ki=3.50, kd=0.0024, dt=dt, output_limits=(0, 24))  # 24V 最大电压

    setpoint = 500.0  # 目标转速（rad/s）

    times, speeds, voltages,  iis = [], [], [] , []
    vffs = []
    vpids = []
    


    for i in range(steps):
        t = i * dt
        omega = motor.get_speed()  # 当前转速 

        Vff =  motor.get_Ke() * setpoint
        Vpid = pid.compute(setpoint, omega)
        voltage = np.clip(Vff + Vpid, 0, 24)
        i , omega_next = motor.update(voltage, dt)  # 更新系统状态

        vffs.append(Vff)
        vpids.append(Vpid)
        # 记录
        times.append(t)
        speeds.append(omega)
        voltages.append(voltage)
        iis.append(i)
    # 绘图
    plt.figure()
    plt.plot(times, vffs, label='Vff')
    plt.plot(times, vpids, label='Vpid')
    plt.plot(times, voltages, label='Vff + Vpid (Clipped)')
    plt.legend()
    plt.grid()
    plt.title("Voltage Components")


    plt.figure(figsize=(10,5))

    plt.subplot(3,1,1)
    plt.plot(times, speeds, label='Speed (rad/s)')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('ω (rad/s)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot(times, iis, label='i (mA)')
    plt.ylabel('mA')
    plt.legend()
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot(times, voltages, label='Voltage Input (V)')
    plt.ylabel('V (Volt)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
