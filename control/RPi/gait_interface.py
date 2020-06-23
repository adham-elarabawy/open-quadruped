import math
import time

from lib.Bezier import Bezier
from lib.GaitPlanner import GaitPlanner
from lib.LLC_Interface import LLC_Interface

llc = LLC_Interface()

y = 80
base_height = 150
# L_span = 40
v_d = 400
alpha = 0


T_swing = 0.3

L_span = T_swing * v_d / 2

print(L_span)

T_stride = 2 * L_span / v_d

x_shift = -25

planner = GaitPlanner(T_stride, T_swing, [0, 0.5, 0.5, 0])
swing = Bezier(Bezier.get_cp_from_param(
    L_span=L_span, base_height=base_height, clearance=20))
stride = Bezier(
    [[L_span, base_height], [0, base_height + alpha], [-L_span, base_height]])

start_time = time.time()
while not False:
    fps_start_time = time.time()
    for i in range(0, 4):
        signal = planner.signal_sample(time.time() - start_time, i)

        if signal[0] == 0:
            x, z = stride.sample_bezier(signal[1])
        if signal[0] == 1:
            x, z = swing.sample_bezier(signal[1])

        llc.add_to_buffer(i, round(x + x_shift, 1), round(y, 1), round(z, 1))
    llc.send_buffer()
    print(
        f'fps: {round(1/(time.time() - fps_start_time), 1)}', end='\r')
