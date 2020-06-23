from math import factorial

import numpy as np


class Bezier:
    def __init__(self, cp):
        self.cp = cp
        self.n = len(self.cp) - 1
        self.coeff = []

        for i, point in enumerate(self.cp):
            self.coeff.append(Bezier.bin_coeff(self.n, i))

    @staticmethod
    def bin_coeff(n, i):
        return factorial(n) / (factorial(i) * factorial(n - i))

    def sample_bezier(self, t):
        x = 0
        y = 0
        for i, point in enumerate(self.cp):
            x += self.coeff[i] * (1 - t)**(self.n - i) * t**i * point[0]
            y += self.coeff[i] * (1 - t)**(self.n - i) * t**i * point[1]
        return [x, y]

    @staticmethod
    def get_cp_from_param(L_span=50, base_height=150, clearance=0):
        x_scaling_factor = L_span / 200
        y_scaling_factor = base_height / 500
        x_center = 0
        cp = [
            [x_center - L_span, base_height],
            [x_center - L_span - 80.5 * x_scaling_factor, base_height],
            [x_center - L_span - 100 * x_scaling_factor,
                base_height - 138.9 * y_scaling_factor - clearance],
            [x_center - L_span - 100 * x_scaling_factor,
                base_height - 138.9 * y_scaling_factor - clearance],
            [x_center - L_span - 100 * x_scaling_factor,
                base_height - 138.9 * y_scaling_factor - clearance],
            [x_center, base_height - 138.9 * y_scaling_factor - clearance],
            [x_center, base_height - 138.9 * y_scaling_factor - clearance],
            [x_center, base_height - 178.6 * y_scaling_factor - clearance],
            [x_center + L_span + 103.2 * x_scaling_factor,
                base_height - 178.6 * y_scaling_factor - clearance],
            [x_center + L_span + 103.2 * x_scaling_factor,
                base_height - 178.6 * y_scaling_factor - clearance],
            [x_center + L_span + 82.6 * x_scaling_factor, base_height],
            [x_center + L_span, base_height]
        ]

        return cp
