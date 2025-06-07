import matplotlib.pyplot as plt
from pid import PID
from bldc_model import BLDC_MODEL
from strategy import Strategy
import numpy as np

def sixstep_sinusoidal(mode):
    dt = 0.001
    total_time = 0.5
    steps = int(total_time / dt)

    motor = BLDC_MODEL()
    control = Strategy()
    control.set_mode(mode)
    control.set_voltage(220, mode)

    pid = PID(kp=5, ki=0.8, kd=0.05, dt=dt, output_limits=(0, 220))
    setpoint = 314.0

    times, speeds, Ia, Ib, Ic, Ea, Eb, Ec = [], [], [], [], [], [], [], []

    omega = motor.get_state()["omega"]
    theta = motor.get_state()["theta"]

    for i in range(steps):
        t = i * dt
        voltage_pid = pid.compute(setpoint, omega)
        control.set_voltage(voltage_pid, mode)
        va, vb, vc = control.compute_voltage(theta, omega)

        if np.any(np.isnan([va, vb, vc])):
            print(f"⚠️ NaN in control voltage: va={va}, vb={vb}, vc={vc}")
            break

        state = motor.update(va, vb, vc, dt, mode)
        theta = state["theta"]
        omega = state["omega"]

        if np.any(np.isnan([theta, omega])):
            print(f"⚠️ NaN Detected in motor update: θ={theta}, ω={omega}")
            break

        times.append(t)
        speeds.append(omega)
        Ia.append(state["i_a"])
        Ib.append(state["i_b"])
        Ic.append(state["i_c"])
        Ea.append(va)
        Eb.append(vb)
        Ec.append(vc)

    plot_results(times, speeds, Ia, Ib, Ic, Ea, Eb, Ec, setpoint)

def foc():
    dt = 0.001
    total_time = 0.5
    steps = int(total_time / dt)

    motor = BLDC_MODEL()
    control = Strategy()
    control.set_mode('foc')
    control.set_voltage(220, 'foc')
    control.set_direction('reverse')
    setpoint = abs(314.0)

    va, vb, vc = 0.0, 0.0, 0.0
    times, speeds, Ia, Ib, Ic, Ea, Eb, Ec = [], [], [], [], [], [], [], []

    for i in range(steps):
        t = i * dt
        state = motor.update(va, vb, vc, dt, 'foc')
        ia, ib, ic = state["i_a"], state["i_b"], state["i_c"]
        theta = state["theta"]
        omega = state["omega"]

        if np.any(np.isnan([theta, omega, ia, ib, ic])):
            print(f"⚠️ NaN Detected in motor: θ={theta}, ω={omega}, ia={ia}, ib={ib}, ic={ic}")
            break

        va, vb, vc = control.compute_voltage(theta, omega, setpoint, ia, ib, ic)

        if np.any(np.isnan([va, vb, vc])):
            print(f"⚠️ NaN in control voltage: va={va}, vb={vb}, vc={vc}")
            break

        times.append(t)
        speeds.append(omega)
        Ia.append(ia)
        Ib.append(ib)
        Ic.append(ic)
        Ea.append(va)
        Eb.append(vb)
        Ec.append(vc)

    plot_results(times, speeds, Ia, Ib, Ic, Ea, Eb, Ec, setpoint)

def plot_results(times, speeds, Ia, Ib, Ic, Ea, Eb, Ec, setpoint):
    plt.figure(figsize=(10, 5))
    plt.plot(times, Ia, label='i_a (A)')
    plt.plot(times, Ib, label='i_b (A)')
    plt.plot(times, Ic, label='i_c (A)')
    plt.title("Three-phase Currents")
    plt.ylabel('Current (A)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.figure(figsize=(10, 5))
    plt.plot(times, Ea, label='v_a (V)')
    plt.plot(times, Eb, label='v_b (V)')
    plt.plot(times, Ec, label='v_c (V)')
    plt.title("Three-phase Voltages")
    plt.ylabel('Voltage (V)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.figure(figsize=(10, 5))
    plt.title("Motor Speed Response")
    plt.plot(times, speeds, label='ω (rad/s)')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('Speed ω (rad/s)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    # mode = 'hall'
    # mode = 'sinusoidal'
    # sixstep_sinusoidal(mode)
    foc()

if __name__ == '__main__':
    main()
