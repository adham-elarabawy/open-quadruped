import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pynput import keyboard

from IK_Engine import Quadruped

# Setting up 3D matplotlib figure
fig = plt.figure()
ax = Axes3D(fig)
# ax.set_aspect("equal")

x = y = z = yaw = pitch = roll = 0

WINDOW_SIZE = 500
ANIMATE_INTERVAL = 50
start_height = 170
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(-start_height, WINDOW_SIZE - start_height)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')

# Setting up Quadruped
robot = Quadruped(ax=ax, origin=(0, 0, 0), height=start_height)

switch = 'x'
buffer = 0
inc = 5


def on_press(key):
    global switch, buffer, inc, x, y, z, yaw, pitch, roll
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['1']:
        x = y = z = yaw = pitch = roll = 0
    if k in ['x']:
        switch = 'x'
    if k in ['y']:
        switch = 'y'
    if k in ['z']:
        switch = 'z'
    if k in ['a']:
        switch = 'a'
    if k in ['p']:
        switch = 'p'
    if k in ['r']:
        switch = 'r'
    if k in ['up']:
        buffer = inc
    if k in ['down']:
        buffer = -inc
    if k in ['up', 'down']:
        if switch == 'x':
            x += buffer
        if switch == 'y':
            y += buffer
        if switch == 'z':
            z += buffer
        if switch == 'a':
            yaw += buffer
        if switch == 'p':
            pitch += buffer
        if switch == 'r':
            roll += buffer


def setup():
    ax.clear()
    ax.set_aspect("equal")

    ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
    ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
    ax.set_zlim3d(-start_height, WINDOW_SIZE - start_height)

    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    ax.set_zlabel('z (mm)')


def animate(i):
    global x, y, z, yaw, pitch, roll
    setup()
    # Going to starting pose
    robot.start_position()
    # Shifting robot pose in cartesian system x-y-z (body-relative)
    robot.shift_body_xyz(x, y, z)
    # Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
    robot.shift_body_rotation(math.radians(
        yaw), math.radians(pitch), math.radians(roll))

    robot.draw_body()
    robot.draw_legs()

    # print(i * ANIMATE_INTERVAL / 1000)


listener = keyboard.Listener(on_press=on_press)
listener.start()  # start to listen on a separate thread

ani = animation.FuncAnimation(fig, animate, interval=ANIMATE_INTERVAL)
plt.show()
