import math

class LegIKModel:
    def __init__(self, upper, lower, off0, off1):
        """Leg Inverse Kinematics Model. Based on Adham Elarabawy's paper: www.adham-e.dev/papers

        args: upper, lower, off0, off1
        degs: length of the upper arm (pivot to pivot in mm), length of the lower arm (pivot to pivot in mm), off0 from paper, off1 from paper
        """
        self.upper = upper
        self.lower = lower
        self.off0 = off0
        self.off1 = off1

    def ja_from_htf_vecs(self, htf_vecs):
        joint_angles = []
        try:
            for i, (x, y, z) in enumerate(htf_vecs):
                h1 = math.sqrt(self.off0**2 + self.off1**2)
                h2 = math.sqrt(z**2 + y**2)
                alpha_0 = math.atan(y / z)
                alpha_1 = math.atan(self.off1 / self.off0)
                alpha_2 = math.atan(self.off0 / self.off1)
                alpha_3 = math.asin(
                    h1 * math.sin(alpha_2 + math.radians(90)) / h2)
                alpha_4 = math.radians(
                    180) - (alpha_3 + alpha_2 + math.radians(90))
                alpha_5 = alpha_1 - alpha_4
                theta_h = alpha_0 - alpha_5

                r0 = h1 * math.sin(alpha_4) / math.sin(alpha_3)
                h = math.sqrt(r0**2 + x**2)
                phi = math.asin(x / h)
                theta_s = math.acos(
                    (h**2 + self.upper**2 - self.lower**2) / (2 * h * self.upper)) - phi
                theta_w = math.acos((self.lower**2 + self.upper **
                                     2 - h**2) / (2 * self.lower * self.upper))
                joint_angles.append([theta_h, theta_s, theta_w])
        except:
            print("Out of Bounds.")

        return joint_angles
