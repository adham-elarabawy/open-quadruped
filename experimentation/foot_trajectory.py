import matplotlib.pyplot as plt
import numpy as np

from gen_suite import Path, Pose

# path parameters
contact_length = 30  # mm
dip_height = 20  # mm
dip_increment_0 = 10  # mm
dip_increment_1 = 5  # mm


waypoints = [Pose(0, 0, 0),
             Pose(contact_length / 2, 0, 0),
             Pose(contact_length / 4, dip_increment_0, 165),
             Pose(-2 * contact_length / 3, dip_increment_1, -150),
             Pose(-contact_length / 2, 0, 0),
             Pose(0, 0, 0)]
path = Path(waypoints)
x, y = path.get_plot_values()

plt.axes().set_aspect('equal')
plt.title("Quadruped Foot Path: Quintic Hermite Splines")
plt.plot(x, y)
plt.plot([pose.x for pose in waypoints], [pose.y for pose in waypoints], 'ro')
plt.show()
