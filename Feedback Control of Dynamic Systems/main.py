import matplotlib.pyplot as plt
from pid import PID
from thermal import ThermalSystem

def simulate(setpoint=70, dt=0.1, sim_time=60):
    pid    = PID(kp=5.0, ki=0.5, kd=1.0, dt=dt, output_limits=(0,100))
    system = ThermalSystem()
    steps  = int(sim_time / dt)

    times, temps, powers = [], [], []
    for i in range(steps):
        t = i * dt
        temp = system.temperature
        power = pid.compute(setpoint, temp)
        temp = system.update(power, dt)
        times.append(t); temps.append(temp); powers.append(power)

    plt.figure(figsize=(10,5))
    plt.subplot(2,1,1)
    plt.plot(times, temps, label='Temperature')
    plt.axhline(setpoint, linestyle='--', label='Setpoint')
    plt.ylabel('°C'); plt.legend(); plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(times, powers, label='Heater Power (%)')
    plt.xlabel('Time (s)'); plt.ylabel('%'); plt.legend(); plt.grid(True)
    plt.tight_layout(); plt.show()

if __name__ == '__main__':
    simulate()
