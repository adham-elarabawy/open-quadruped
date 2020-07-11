import math
import time

import serial

from approxeng.input.selectbinder import ControllerResource
from lib.IK_Engine import Quadruped
from lib.LLC_Interface import LLC_Interface
from pytictoc import TicToc

# Setting up low-level controller interface
llc = LLC_Interface()

# Setting up Quadruped
robot = Quadruped(origin=(0, 0, 0), height=170)

# Setting up performance metrics
t = TicToc()
DEBUG = True

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

                # Going to starting pose
                robot.start_position()
                # Shifting robot pose in cartesian system x-y-z (body-relative)
                robot.shift_body_xyz(0, y_bound * dpad[0], z_bound * dpad[1])
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

                if DEBUG:
                    print(
                        f'fps: {round(1/t.tocvalue(), 1)}, left: {left_joy}, right: {right_joy}, dpad: {dpad}', end='\r')
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        time.sleep(1)
