import numpy as np


class GaitPlanner:
    def __init__(self, T_stance, T_swing, phase_lag):
        self.T_stance = T_stance
        self.T_swing = T_swing
        self.phase_lag = phase_lag
        self.T_stride = self.T_swing + self.T_stance

    def signal_sample(self, time, leg):
        phase_i = self.phase_lag[leg]
        phase_time = phase_i * self.T_stride

        base_time = (time - phase_time) % self.T_stride
        while base_time < 0:
            base_time = self.T_stride - base_time

        if base_time <= self.T_stance:
            # in stride period
            return [0, base_time / self.T_stance]
        else:
            # in swing period
            return [1, (base_time - self.T_stance) / self.T_swing]
