import math
import time

import serial

import xbox

from ../IK_Engine import Quadruped

#FL, FR, BL, BR
#Hip, Shoulder, Width
joint_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

joy = xbox.Joystick()
# Setting up Quadruped
robot = Quadruped(origin=(0, 0, 0), height=170)
x_shift = y_shift = z_shift = yaw_shift = roll_shift = 0
x_bound = y_bound = z_bound = 50
roll_bound = 20
dampening_rate = 3
max_speed = 150  # deg/sec

alpha = 0.65
prev_joy_x = prev_joy_y = prev_joy_x_r = 0

deadzone = 0.2

ser = serial.Serial('/dev/ttyS0', 256000)
ser.flush()

while not joy.Back():
    data_received = ""
    line = ""
    start_time = time.time()

    # joystick filtering:

    prefilter_x = -joy.leftX()
    prefilter_y = joy.leftY()
    prefilter_x_r = -joy.rightX()

    if prefilter_x < deadzone and prefilter_x > -deadzone:
        prefilter_x = 0
    if prefilter_y < deadzone and prefilter_y > -deadzone:
        prefilter_y = 0
    if prefilter_x_r < deadzone and prefilter_x_r > -deadzone:
        prefilter_x_r = 0

    joy_x = round(alpha * prev_joy_x + (1 - alpha) * prefilter_x, 1)
    joy_y = round(alpha * prev_joy_y + (1 - alpha) * prefilter_y, 1)
    joy_x_r = round(alpha * prev_joy_x_r + (1 - alpha) * prefilter_x_r, 1)

    prev_joy_x = joy_x
    prev_joy_y = joy_y
    prev_joy_x_r = joy_x_r

    # getting servo positions from arduino
    if ser.inWaiting():
        data_received = str(ser.readline().decode('utf-8').rstrip())
        if(len(data_received) > 2):
            leg = data_received[0]
            joint = data_received[1]
            angle = data_received[2:]

            if joint.capitalize() == "H":
                joint_positions[int(leg)][0] = angle
            if joint.capitalize() == "S":
                joint_positions[int(leg)][1] = angle
            if joint.capitalize() == "W":
                joint_positions[int(leg)][2] = angle

    line = str(joint_positions)

    x_shift = round(x_bound * joy_y, 1)
    y_shift = round(y_bound * joy_x, 1)
    z_shift += round(dampening_rate * (joy.dpadUp() - joy.dpadDown()), 1)
    roll_shift = round(roll_bound * joy_x_r, 1)
    yaw_shift += round(dampening_rate / 5 *
                       (joy.dpadRight() - joy.dpadLeft()), 1)
    if (z_shift > z_bound):
        z_shift = z_bound
    if (z_shift < -z_bound):
        z_shift = -z_bound
    if (joy.Y()):
        x_shift = y_shift = z_shift = yaw_shift = roll_shift = 0
    # Going to starting pose
    robot.start_position()
    # Shifting robot pose in cartesian system x-y-z (body-relative)
    robot.shift_body_xyz(x_shift, y_shift, z_shift)
    # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
    robot.shift_body_rotation(math.radians(
        yaw_shift), math.radians(0), math.radians(roll_shift))

    strings_to_send = []
    fl = robot.legs[2]
    fr = robot.legs[1]
    bl = robot.legs[3]
    br = robot.legs[0]
    legs = [fl, fr, bl, br]
    key = ["FL", "FR", "BL", "BR"]

    factor = 1

    for i, leg in enumerate(legs):

        desired_hip = round(math.degrees(-leg.hip_rad), 1)
        desired_shoulder = round(math.degrees(leg.shoulder_rad), 1)
        desired_wrist = round(math.degrees(leg.wrist_rad), 1)

        current_hip = int(joint_positions[i][0])
        current_shoulder = int(joint_positions[i][1])
        current_wrist = int(joint_positions[i][2])

        differences = [abs(desired_hip - current_hip),
                       abs(desired_shoulder - current_shoulder),
                       abs(desired_wrist - current_wrist)]
        speeds = [0, 0, 0]

        limiting_factor = max(differences)

        for i0, difference in enumerate(differences):
            speeds[i0] = max_speed * difference / limiting_factor

        strings_to_send.append(
            f'{key[i]},H,{desired_hip},{speeds[0]}\n')
        strings_to_send.append(
            f'{key[i]},S,{desired_shoulder},{speeds[1]}\n')
        strings_to_send.append(
            f'{key[i]},W,{desired_wrist},{speeds[2]}\n')

    for message in strings_to_send:
        ser.write(message.encode('utf-8'))
    print(
        f'fps: {round(1/(time.time() - start_time), 1)}, data: {line}')


joy.close()
