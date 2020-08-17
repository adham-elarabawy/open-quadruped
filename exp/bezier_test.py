from math import factorial

import matplotlib.pyplot as plt
import numpy as np

res = 1000

cp = [[100, 100],
      [50, 300],
      [300, 500],
      [500, 100],
      [600, 300],
      [800, 200]]


def bin_coeff(n, i):
    return factorial(n) / (factorial(i) * factorial(n - i))


def sample_bezier(t, cp):
    x = 0
    y = 0
    n = len(cp) - 1
    for i, point in enumerate(cp):
        x += bin_coeff(n, i) * (1 - t)**(n - i) * t**i * point[0]
        y += bin_coeff(n, i) * (1 - t)**(n - i) * t**i * point[1]
    return [x, y]


sample_space = np.linspace(0, 1, res)
samples = [sample_bezier(sample_t, cp) for sample_t in sample_space]
plt.scatter([point[0] for point in cp], [point[1]
                                         for point in cp], color='red')
plt.plot([sample[0] for sample in samples], [sample[1] for sample in samples])

plt.xlabel('x (mm)')
plt.ylabel('y (mm)')

plt.show()
