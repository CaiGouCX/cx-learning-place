import argparse
import matplotlib.pyplot as plt
from pid import PID
from thermal import ThermalSystem
from FirstOrderThermalSystem import FirstOrderThermalSystem
from ThermalSystem2ndOrder import ThermalSystem2ndOrder


def simulate(systemtype):
    setpoint=70
    dt=0.1
    # 系统类型：
    # 1 -> 一阶
    # 2 -> 一阶滞后
    # 3 -> 二阶

    if systemtype == 1:
        MAX_POWER = 100
        pid = PID(kp=10, ki=0.5, kd=0.1, dt=dt, output_limits=(0, 100))
        system = ThermalSystem()
        steps  = int(60 / dt)
    elif systemtype == 2:
        MAX_POWER = 200
        pid = PID(kp=6.0, ki=0.1, kd=0.01, dt=dt, output_limits=(0, 100))
        system = FirstOrderThermalSystem()
        steps  = int(100 / dt)
    elif systemtype == 3:
        MAX_POWER = 4570
        pid = PID(kp=27.655, ki=0.1074, kd=0, dt=0.1, output_limits=(0, 100))
        system = ThermalSystem2ndOrder()
        steps  = int(15000 / dt)
    else:
        raise ValueError("Unsupported system type. Choose 1, 2, or 3.")


    '''times, temps, powers = [], [], []
    for i in range(steps):
        t = i * dt
        temp = system.temperature
        power = pid.compute(setpoint, temp)
        temp = system.update(power, dt)
        times.append(t)
        temps.append(temp)
        powers.append(power)'''

    # 正确的控制循环调用方式（单位匹配版）
    times, temps, powers = [], [], []
    for i in range(steps):
        t = i * dt
        # 当前温度（原始物理量）
        temp = system.temperature
        # 归一化温度：例如 25°C -> 0.0，70°C -> 1.0
        norm_temp = (temp - 25.0) / (setpoint - 25.0)

        # PID 控制目标是归一化值 1.0（表示期望达到 70°C）
        pid_output = pid.compute(1.0,norm_temp)  # 输出范围应该是 0~1
        # 限制 PID 输出在 0~1 区间，防止实际功率过大
        power = max(0.0, min(pid_output, 1.0)) * MAX_POWER
        print(power)
        # 更新系统状态
        temp = system.update(power, dt)
        # 记录数据
        times.append(t)
        temps.append(temp)
        powers.append((power / MAX_POWER) *100)


    plt.figure(figsize=(10,5))
    plt.subplot(2,1,1)
    plt.plot(times, temps, label='Temperature')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('°C'); plt.legend(); plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(times, powers, label='Heater Power')
    plt.xlabel('Time (s)'); plt.ylabel('%'); plt.legend(); plt.grid(True)
    plt.tight_layout(); plt.show()



def main():
    # 1. 设置参数解析器
    parser = argparse.ArgumentParser(description="热系统仿真程序")
    parser.add_argument("-o", "--order", type=int, choices=[1, 2, 3], required=True,
                        help="选择系统阶数 (1: 一阶, 2: 一阶滞后,3: 二阶)")
    parser.add_argument("-t", "--time", type=float, default=10.0,
                        help="仿真总时间（默认：60秒）")
    # 一阶滞后系统特有参数

    # 二阶系统特有参数
    parser.add_argument("--zeta", type=float, default=0.7,
                        help="二阶系统阻尼比（默认：0.7）")
    parser.add_argument("--omega_n", type=float, default=1.0,
                        help="二阶系统自然频率（默认：1.0 rad/s）")
    
    args = parser.parse_args()

    # 2. 根据参数调用对应函数
    if args.order == 1:
         simulate(1)
    elif args.order == 2:
         simulate(2)
    elif args.order == 3:
         print("3")
         simulate(3)
if __name__ == '__main__':
    main()
