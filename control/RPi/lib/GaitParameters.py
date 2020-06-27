class GaitParameters:
    def __init__(self, phase_lag, T_swing, L_span, v_d, penetration_alpha, base_height, y, x_shift, clearance):
        self.phase_lag = phase_lag
        self.T_swing = T_swing
        self.T_stance = 2 * L_span / v_d
        self.L_span = L_span
        self.v_d = v_d
        self.penetration_alpha = penetration_alpha
        self.base_height = base_height
        self.y = y
        self.x_shift = x_shift
        self.clearance = clearance
