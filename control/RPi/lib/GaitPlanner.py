import numpy as np


class GaitPlanner:
    def __init__(self, T_stride, T_swing, phase_lag):
        self.T_stride = T_stride
        self.T_swing = T_swing
        self.phase_lag = phase_lag
        self.base_period = self.T_swing + self.T_stride

    def signal_sample(self, time, leg):
        base_time = (time - self.phase_lag[leg]) % self.base_period
        while base_time < 0:
            base_time = self.base_period - base_time

        if base_time <= self.T_stride:
            # in stride period
            return [0, base_time / self.T_stride]
        else:
            # in swing period
            return [1, (base_time - self.T_stride) / self.T_swing]
