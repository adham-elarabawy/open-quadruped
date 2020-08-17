from body_ik import BodyIKModel
from leg_ik import LegIKModel
import math
import matplotlib.pyplot as plt
import numpy as np

body_model = BodyIKModel(76.655, 229.3, 130)
leg_model = LegIKModel(109.868, 144.580, 11.369, 63.763)

body_model.transform(math.radians(0), math.radians(0), math.radians(0))
htf_vecs = body_model.get_htf_vectors()

ja = leg_model.ja_from_htf_vecs(htf_vecs)

print(ja)

idx = [0, 1, 3, 2, 0]

plt.plot([point[0] for point in body_model.leg_points], [point[1] for point in body_model.leg_points], 'ro')
plt.plot([point[0] for point in body_model.body_points[idx]], [point[1] for point in body_model.body_points[idx]])
plt.axis('equal')
plt.show()
