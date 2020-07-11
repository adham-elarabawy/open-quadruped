import math
import time

from lib.Bezier import Bezier
from lib.GaitParameters import GaitParameters
from lib.GaitPlanner import GaitPlanner
from lib.LLC_Interface import LLC_Interface

# GAITS
#gait_name = GaitParameters(phase_lag, T_swing, L_span, v_d, penetration_alpha, base_height, y, x_shift, clearance)
crawl = GaitParameters([0, 0.5, 0.75, 0.25], 0.4, 70, 50, 5, 160, 55, -25, 25)
trot = GaitParameters([0, 0.5, 0.5, 0], 0.3, 50, 100, 5, 150, 55, -40, 5)


llc = LLC_Interface()

# **CHANGE SELECTED GAIT HERE ** #
gait = trot

planner = GaitPlanner(gait.T_stance, gait.T_swing, gait.phase_lag)
swing = Bezier(Bezier.get_cp_from_param(
    L_span=gait.L_span, base_height=gait.base_height, clearance=gait.clearance))
stance = Bezier(
    [[gait.L_span, gait.base_height], [0, gait.base_height + gait.penetration_alpha], [-gait.L_span, gait.base_height]])

start_time = time.time()
while not False:
    fps_start_time = time.time()
    for i in range(0, 4):
        signal = planner.signal_sample(time.time() - start_time, i)

        if signal[0] == 0:
            x, z = stance.sample_bezier(signal[1])
        if signal[0] == 1:
            x, z = swing.sample_bezier(signal[1])

        llc.add_to_buffer(i, round(x + gait.x_shift, 1),
                          round(gait.y, 1), round(z, 1))
    llc.send_buffer()
    print(
        f'fps: {round(1/(time.time() - fps_start_time), 1)}', end='\r')
