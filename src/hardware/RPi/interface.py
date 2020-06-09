import math
import time

import serial

import xbox
from IK_Engine import Quadruped

joy = xbox.Joystick()
# Setting up Quadruped
robot = Quadruped(origin=(0, 0, 0), height=170)
x_shift = y_shift = z_shift = yaw_shift = 0
x_bound = y_bound = z_bound = 50
dampening_rate = 3
temp_speed = 200

alpha = 0.8
prev_joy_x = prev_joy_y = 0

deadzone = 0.1

ser = serial.Serial('/dev/ttyS0', 115200)
ser.flush()

while not joy.Back():

    line = ""
    start_time = time.time()

    # joystick filtering:

    prefilter_x = joy.leftX()
    prefilter_y = joy.leftY()

    if prefilter_x < deadzone and prefilter_x > -deadzone:
        prefilter_x = 0
    if prefilter_y < deadzone and prefilter_y > -deadzone:
        prefilter_y = 0

    joy_x = alpha * prev_joy_x + (1 - alpha) * prefilter_x
    joy_y = alpha * prev_joy_y + (1 - alpha) * prefilter_y

    prev_joy_x = joy_x
    prev_joy_y = joy_y

    # getting servo positions from arduino
    # if ser.inWaiting():
    #     line += ser.readline().decode('utf-8').rstrip()

    x_shift = x_bound * joy_y
    y_shift = y_bound * joy_x
    z_shift += dampening_rate * (joy.dpadUp() - joy.dpadDown())
    yaw_shift += dampening_rate / 5 * (joy.dpadRight() - joy.dpadLeft())
    if (z_shift > z_bound):
        z_shift = z_bound
    if (z_shift < -z_bound):
        z_shift = -z_bound
    if (joy.Y()):
        x_shift = y_shift = z_shift = yaw_shift = 0
    # Going to starting pose
    robot.start_position()
    # Shifting robot pose in cartesian system x-y-z (body-relative)
    robot.shift_body_xyz(x_shift, y_shift, z_shift)
    # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
    robot.shift_body_rotation(math.radians(
        yaw_shift), math.radians(0), math.radians(0))

    strings_to_send = []
    fl = robot.legs[2]
    fr = robot.legs[1]
    bl = robot.legs[3]
    br = robot.legs[0]
    legs = [fl, fr, bl, br]
    key = ["FL", "FR", "BL", "BR"]

    for i, leg in enumerate(legs):
        # Front Left
        strings_to_send.append(
            f'{key[i]},H,{round(math.degrees(-leg.hip_rad),1)},{temp_speed}\n')
        strings_to_send.append(
            f'{key[i]},S,{round(math.degrees(leg.shoulder_rad),1)},{temp_speed}\n')
        strings_to_send.append(
            f'{key[i]},W,{round(math.degrees(leg.wrist_rad),1)},{temp_speed}\n')

    for message in strings_to_send:
        ser.write(message.encode('utf-8'))
    print(
        f'fps: {round(1/(time.time() - start_time), 1)}, positions: {line}')

joy.close()
