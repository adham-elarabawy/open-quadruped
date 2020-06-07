import xbox
from IK_Engine import Quadruped

# Setting up Quadruped
robot = Quadruped(ax, origin=(0, 0, 0), height=170)
# Going to starting pose
robot.start_position()
# Shifting robot pose in cartesian system x-y-z (body-relative)
robot.shift_body_xyz(30, 0, 0)
# Shifting robot pose in Euler Angles yaw-pitch-roll (body-relative)
robot.shift_body_rotation(math.radians(
    20), math.radians(-10), math.radians(0))
