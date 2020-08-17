import math
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IK_Engine import Quadruped

sys.path.append("../control")



# Setting up 3D matplotlib figure
fig = plt.figure()
ax = Axes3D(fig)
ax.set_aspect("equal")

start_height = 170
WINDOW_SIZE = 500
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(-start_height, WINDOW_SIZE - start_height)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')

# Setting up Quadruped
robot = Quadruped(ax=ax, origin=(0, 0, 0), height=start_height)
# Going to starting pose
robot.start_position()
# Shifting robot pose in cartesian system x-y-z (body-relative)
robot.shift_body_xyz(30, 0, 0)
# Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
robot.shift_body_rotation(math.radians(
    20), math.radians(-10), math.radians(0))


robot.draw_body()
robot.draw_legs()

plt.show()
