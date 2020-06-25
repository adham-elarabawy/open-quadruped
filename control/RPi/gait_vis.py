import math
import time

import matplotlib.pyplot as plt

from lib.Bezier import Bezier
from lib.GaitPlanner import GaitPlanner

x = [0]
y = [-150]

x0 = [0]
y0 = [-150]

plt.ion()
fig, ax = plt.subplots()
sc = ax.scatter(x, y, c='black', label='FL/BR leg')
sc1 = ax.scatter(x0, y0, c='red', label='FR/BL leg')

base_height = 150
L_span = 40
v_d = 80
alpha = 20

T_swing = 0.15
T_stance = 2 * L_span / v_d

print(T_stance)

plt.xlim(-2 * L_span, 2 * L_span)
plt.ylim(-4 / 3 * base_height, 0)
plt.xlabel('x (mm)')
plt.ylabel('z (mm)')
plt.legend(loc="upper left")
plt.title('Outcome Trajectory: Gait Planner + Bezier')
plt.draw()
fig.canvas.draw_idle()

planner = GaitPlanner(T_stance, T_swing, [0, 0.5, 0.5, 0])
swing = Bezier(Bezier.get_cp_from_param(
    L_span=L_span, base_height=base_height, clearance=10))
stance = Bezier(
    [[L_span, base_height], [0, base_height + alpha], [-L_span, base_height]])

start_time = time.time()
while not False:
    fps_start_time = time.time()
    for i in range(0, 4):
        signal = planner.signal_sample(time.time() - start_time, i)

        if signal[0] == 0:
            x_val, z_val = stance.sample_bezier(signal[1])
        if signal[0] == 1:
            x_val, z_val = swing.sample_bezier(signal[1])

        if(i == 0):
            x = [round(x_val, 1)]
            y = [-round(z_val, 1)]
        if(i == 1):
            x0 = [round(x_val, 1)]
            y0 = [-round(z_val, 1)]
            sc.remove()
            sc1.remove()
            sc = ax.scatter(x, y, c='black', label='FL/BR leg')
            sc1 = ax.scatter(x0, y0, c='red', label='FR/BL leg')
            sc2 = ax.plot([-2 * L_span, 2 * L_span],
                          [-base_height, -base_height], c='grey')
            fig.canvas.draw_idle()
            plt.pause(0.005)
    print(
        f'fps: {round(1/(time.time() - fps_start_time), 1)}', end='\r')
