from body_ik import BodyIKModel
import math
import matplotlib.pyplot as plt
import numpy as np

body_model = BodyIKModel(76.655, 229.3, 145)
body_model.transform(math.radians(20), math.radians(0), math.radians(0))
htf_vecs = body_model.get_htf_vectors()

print("htf vectors:")
print(htf_vecs)
print("body points:")
print(body_model.body_points)
print("leg_points:")
print(body_model.leg_points)

idx = [0, 1, 3, 2, 0]

plt.plot([point[0] for point in body_model.leg_points], [point[1] for point in body_model.leg_points], 'ro')
plt.plot([point[0] for point in body_model.body_points[idx]], [point[1] for point in body_model.body_points[idx]])
plt.axis('equal')
plt.show()
