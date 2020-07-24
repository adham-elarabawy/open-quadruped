import math
import time

import serial

from approxeng.input.selectbinder import ControllerResource
from lib.Bezier import Bezier
from lib.GaitParameters import GaitParameters
from lib.GaitPlanner import GaitPlanner
from lib.IK_Engine import Quadruped
from lib.LLC_Interface import LLC_Interface
from pytictoc import TicToc

# Setting up low-level controller interface
llc = LLC_Interface()

# Setting up Quadruped
robot = Quadruped(origin=(0, 0, 0), height=170)
mode = 'pose'

# Setting up performance metrics
t = TicToc()
DEBUG = False  # whether or not to show fps/joystick vals

# Gaits
#gait_name = GaitParameters(phase_lag, T_swing, L_span, v_d, penetration_alpha, base_height, y, x_shift, clearance)
crawl = GaitParameters([0, 0.5, 0.75, 0.25], 0.4, 70, 50, 5, 160, 55, -25, 25)
trot = GaitParameters([0, 0.5, 0.5, 0], 0.3, 50, 100, 5, 150, 55, -40, 5)
fast_trot = GaitParameters([0, 0.5, 0.5, 0], 0.2,
                           40, 130, 5, 150, 60, -60, 10)

# **CHANGE SELECTED GAIT HERE ** #
gait = fast_trot

# Setting up Gait Planner & Trajectories
planner = GaitPlanner(gait.T_stance, gait.T_swing, gait.phase_lag)
swing = Bezier(Bezier.get_cp_from_param(
    L_span=gait.L_span, base_height=gait.base_height, clearance=gait.clearance))
stance = Bezier(
    [[gait.L_span, gait.base_height], [0, gait.base_height + gait.penetration_alpha], [-gait.L_span, gait.base_height]])

# Limits
y_bound = 50
z_bound = 60
pitch_bound = 18
yaw_bound = 30

# Controller Values
left_joy = [0, 0]
right_joy = [0, 0]
dpad = [0, 0]

# Controller Parameters
dpad_res = 5  # how many clicks in each direction to reach axis bound


while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            while joystick.connected:
                t.tic()

                # Instance of approxeng.input.ButtonPresses
                presses = joystick.check_presses()
                left_joy = [joystick.lx, joystick.ly]
                right_joy = [joystick.rx, joystick.ry]
                if joystick.presses.dleft:
                    if dpad[0] > -1:
                        dpad[0] -= 1 / dpad_res
                if joystick.presses.dright:
                    if dpad[0] < 1:
                        dpad[0] += 1 / dpad_res
                if joystick.presses.ddown:
                    if dpad[1] > -1:
                        dpad[1] -= 1 / dpad_res
                if joystick.presses.dup:
                    if dpad[1] < 1:
                        dpad[1] += 1 / dpad_res

                if joystick.presses.triangle:
                    left_joy = [0, 0]
                    right_joy = [0, 0]
                    dpad = [0, 0]

                if joystick.presses.circle:
                    print('\nExiting...')
                    exit()

                if(joystick.r1 == None):
                    mode = 'pose'
                    # BODY POSE MODE

                    # Going to starting pose
                    robot.start_position()
                    # Shifting robot pose in cartesian system x-y-z (body-relative)
                    robot.shift_body_xyz(
                        0, y_bound * dpad[0], z_bound * dpad[1])
                    # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
                    robot.shift_body_rotation(math.radians(
                        yaw_bound * left_joy[0]), math.radians(pitch_bound * left_joy[1]), 0)

                    fl = robot.legs[2]
                    fr = robot.legs[1]
                    bl = robot.legs[3]
                    br = robot.legs[0]
                    legs = [fl, fr, bl, br]

                    for i, leg in enumerate(legs):

                        x = round(leg.x, 1)
                        y = round(leg.y, 1)
                        z = round(leg.z, 1)

                        llc.add_to_buffer(i, x, y, z)

                    llc.send_buffer()

                    # For the gait planner (for when we switch from pose to walk mode)
                    gait_start_time = time.time()
                else:
                    # MOVEMENT MODE
                    mode = 'walk'
                    for i in range(0, 4):
                        signal = planner.signal_sample(
                            time.time() - gait_start_time, i)

                        if signal[0] == 0:
                            x, z = stance.sample_bezier(signal[1])
                        if signal[0] == 1:
                            x, z = swing.sample_bezier(signal[1])

                        if right_joy[1] < -0.2:
                            theta = 180
                        else:
                            theta = right_joy[0] * 90

                        x, y, z = Bezier.rotateAboutZ(x, z, theta)

                        llc.add_to_buffer(i, round(x + gait.x_shift, 1),
                                          round(y + gait.y, 1), round(z, 1))
                    llc.send_buffer()
                if DEBUG:
                    print(
                        f'fps: {round(1/t.tocvalue(), 1)}, mode: {mode}', end='\r')
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        time.sleep(1)
