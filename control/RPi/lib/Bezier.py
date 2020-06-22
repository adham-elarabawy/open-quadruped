from math import factorial

import numpy as np


class Bezier:
    def __init__(self, cp):
        self.cp = cp
        self.n = len(self.cp) - 1
        self.coeff = []

        for i, point in enumerate(self.cp):
            coeff.append(Bezier.bin_coeff(self.n, i))

    @staticmethod
    def bin_coeff(n, i):
        return factorial(n) / (factorial(i) * factorial(n - i))

    def sample_bezier(self, t):
        x = 0
        y = 0
        for i, point in enumerate(self.cp):
            x += self.coeff[i] * (1 - t)**(n - i) * t**i * point[0]
            y += self.coeff[i] * (1 - t)**(n - i) * t**i * point[1]
        return [x, y]
