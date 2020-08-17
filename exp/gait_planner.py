import matplotlib.pyplot as plt
import numpy as np

L_span = 40  # 0.5 * contact distance (mm)
v_d = 50  # desired linear vel (mm/s)

T_stance = 2 * L_span / v_d  # stance period
T_swing = 0.25  # swing period

# phase lag from reference leg 0 (FL) in seconds
phase_lag = [0, T_swing, 2 * T_swing, 3 * T_swing]

colors = ['black', 'blue', 'red', 'green']
label = ['FL', 'FR', 'BL', 'BR']

time_window = (T_stance + T_swing) + max(phase_lag)

num_samples = 1000  # increase for more accurate matplotlib visualization


def signal_sample(time, phase_lag, T_stance, T_swing):
    base_period = T_stance + T_swing
    base_time = (time - phase_lag) % base_period
    while base_time < 0:
        base_time = base_period - base_time

    if base_time <= T_stance:
        return base_time / T_stance
    else:
        return (base_time - T_stance) / T_swing


sampling = np.linspace(0, time_window, num_samples)

for index, color in enumerate(colors):
    plt.plot(sampling, [signal_sample(i, phase_lag[index], T_stance, T_swing)
                        for i in sampling], color, label=label[index])

plt.title('Gait Planner: Swing-Stance Sawtooth Signal')
plt.xlabel('Time (s)')
plt.ylabel('Normalized Signal')
plt.legend(loc="upper left")
plt.xlim(0, time_window)
plt.ylim(0, 1)
plt.show()
