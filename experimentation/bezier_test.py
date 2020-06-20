import numpy as np

import bezier

nodes = np.asfortranarray([
    [0.0, 0.625, 1.0],
    [0.0, 0.5, 0.5]
])
curve = bezier.Curve(nodes, degree=2)
