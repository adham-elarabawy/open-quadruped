import math
import time

import serial

from lib.IK_Engine import Quadruped
from lib.LLC_Interface import LLC_Interface
from lib.xbox import Joystick

joy = Joystick()
llc = LLC_Interface()

# Setting up Quadruped
robot = Quadruped(origin=(0, 0, 0), height=170)
x_shift = y_shift = z_shift = yaw_shift = roll_shift = 0
x_bound = y_bound = z_bound = 50
roll_bound = 30
pitch_bound = 18
dampening_rate = 1

alpha = 0.7
prev_joy_x = prev_joy_y = prev_joy_x_r = prev_joy_y_r = 0

deadzone = 0.2

while not joy.Back():
    data_received = ""
    line = ""
    start_time = time.time()

    # joystick filtering:

    prefilter_x = -joy.leftX()
    prefilter_y = joy.leftY()
    prefilter_x_r = -joy.rightX()
    prefilter_y_r = joy.rightY()

    if prefilter_x < deadzone and prefilter_x > -deadzone:
        prefilter_x = 0
    if prefilter_y < deadzone and prefilter_y > -deadzone:
        prefilter_y = 0
    if prefilter_x_r < deadzone and prefilter_x_r > -deadzone:
        prefilter_x_r = 0
    if prefilter_y_r < deadzone and prefilter_y_r > -deadzone:
        prefilter_y_r = 0

    joy_x = round(alpha * prev_joy_x + (1 - alpha) * prefilter_x, 1)
    joy_y = round(alpha * prev_joy_y + (1 - alpha) * prefilter_y, 1)
    joy_x_r = round(alpha * prev_joy_x_r + (1 - alpha) * prefilter_x_r, 1)
    joy_y_r = round(alpha * prev_joy_y_r + (1 - alpha) * prefilter_y_r, 1)

    prev_joy_x = joy_x
    prev_joy_y = joy_y
    prev_joy_x_r = joy_x_r
    prev_joy_y_r = joy_y_r

    x_shift = round(x_bound * joy_y, 1)
    y_shift = round(y_bound * joy_x, 1)
    z_shift += round(dampening_rate * (joy.dpadUp() - joy.dpadDown()), 1)
    roll_shift = round(roll_bound * joy_x_r, 1)
    pitch_shift = round(pitch_bound * joy_y_r, 1)
    yaw_shift += round(dampening_rate / 5 *
                       (joy.dpadRight() - joy.dpadLeft()), 1)
    if (z_shift > z_bound):
        z_shift = z_bound
    if (z_shift < -z_bound):
        z_shift = -z_bound
    if (joy.Y()):
        x_shift = y_shift = z_shift = yaw_shift = roll_shift = pitch_shift = 0
    if (joy.B()):
        x_shift = y_shift = yaw_shift = roll_shift = pitch_shift = 0
        z_shift = -140
    # Going to starting pose
    robot.start_position()
    # Shifting robot pose in cartesian system x-y-z (body-relative)
    robot.shift_body_xyz(x_shift, y_shift, z_shift)
    # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
    robot.shift_body_rotation(math.radians(
        yaw_shift), math.radians(pitch_shift), math.radians(roll_shift))

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
    print(
        f'fps: {round(1/(time.time() - start_time), 1)}', end='\r')

joy.close()
