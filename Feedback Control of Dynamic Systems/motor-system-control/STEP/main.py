import matplotlib.pyplot as plt
from strategy import *
from step_model import StepperMotor  # 假设建模类保存在 stepper_motor.py 中

def run_simulation(controller, T_end=0.5):
    Ts = controller.motor.Ts
    steps = int(T_end / Ts)

    time_log = []
    theta_log = []
    omega_log = []

    for i in range(steps):
        t = i * Ts
        _, _, omega, theta = controller.update()
        time_log.append(t)
        theta_log.append(theta)
        omega_log.append(omega)

    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(time_log, theta_log)
    plt.ylabel("Position (rad)")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time_log, omega_log)
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (rad/s)")
    plt.grid(True)

    plt.suptitle(controller.__class__.__name__ + " Simulation")
    plt.tight_layout()
    plt.show()

def main():
    motor = StepperMotor()

    # ✅ 可替换测试策略：
    
    #controller = SineWaveController(motor, V=220.0, freq=50, direction=1)
    #controller = FullStepController(motor, step_rate_hz=300, V=220.0, direction=1)
    #controller = HalfStepController(motor, step_rate_hz=300, V=220.0, direction=1)
    #controller = PositionController(motor, V=220.0)
    #controller.set_target(np.radians(1.8))  # 目标角度：180度

    controller = PIDSpeedController(motor, setpoint_rads=10, V=220.0)

    run_simulation(controller)

if __name__ == '__main__':
    main()
