import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from pytictoc import TicToc

t = TicToc()


class InverseKinematics:

    def __init__(self, forearm, shoulder, hip_offset=(0, 0)):
        self.wrist = forearm
        self.shoulder = shoulder
        self.hip_offset = hip_offset  # z, y

    def from_local_xyz(self, side, x, y, z):
        h1 = math.sqrt(self.hip_offset[0]**2 + self.hip_offset[1]**2)
        h2 = math.sqrt(z**2 + y**2)
        alpha_0 = math.atan(y / z)
        alpha_1 = math.atan(self.hip_offset[1] / self.hip_offset[0])
        alpha_2 = math.atan(self.hip_offset[0] / self.hip_offset[1])
        alpha_3 = math.asin(h1 * math.sin(alpha_2 + math.radians(90)) / h2)
        alpha_4 = math.radians(180) - (alpha_3 + alpha_2 + math.radians(90))
        alpha_5 = alpha_1 - alpha_4
        theta_h = alpha_0 - alpha_5

        r0 = h1 * math.sin(alpha_4) / math.sin(alpha_3)
        h = math.sqrt(r0**2 + x**2)
        phi = math.asin(x / h)
        theta_s = math.acos(
            (h**2 + self.shoulder**2 - self.wrist**2) / (2 * h * self.shoulder)) - phi
        theta_w = math.acos((self.wrist**2 + self.shoulder **
                             2 - h**2) / (2 * self.wrist * self.shoulder))

        if side == 'r':
            return theta_h, theta_s, theta_w
        if side == 'l':
            return -theta_h, theta_s, theta_w
        else:
            raise ValueError(side)


class Leg:
    def __init__(self, origin):
        self.origin = origin
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.origin_z = origin[2]


class Quadruped:

    def __init__(self, ax, body_dim=(30, 15), limb_lengths=(10, 10), offsets=(3, 5)):
        '''
        body_dim: (length, width,thickness) in cm
        limb_lengths: (upper_arm, bottom_arm) in cm
        offsets: (z_offset, y_offset) in cm
        '''
        self.ax = ax
        self.body_dim = body_dim
        self.limb_lengths = limb_lengths
        self.offsets = offsets

        self.ik = InverseKinematics(
            limb_lengths[0], limb_lengths[1], hip_offset=self.offsets)

        self.body = [(-self.body_dim[0] / 2, -self.body_dim[1] / 2, 0),
                     (self.body_dim[0] / 2, -self.body_dim[1] / 2, 0),
                     (self.body_dim[0] / 2, self.body_dim[1] / 2, 0),
                     (-self.body_dim[0] / 2, self.body_dim[1] / 2, 0),
                     (-self.body_dim[0] / 2, -self.body_dim[1] / 2, 0)]

        # back_right_leg, front_right_leg, front_left_leg, back_left_leg
        self.legs = [Leg((-self.body_dim[0] / 2, -self.body_dim[1] / 2, 0)),
                     Leg((self.body_dim[0] / 2, -self.body_dim[1] / 2, 0)),
                     Leg((self.body_dim[0] / 2, self.body_dim[1] / 2, 0)),
                     Leg((-self.body_dim[0] / 2, self.body_dim[1] / 2, 0))]

    @staticmethod
    def add_vector(base_vector, increment):
        assert(len(base_vector) == len(increment))
        return [val + increment[i] for i, val in enumerate(base_vector)]

    @staticmethod
    def subtract_vector(base_vector, increment):
        assert(len(base_vector) == len(increment))
        return [val - increment[i] for i, val in enumerate(base_vector)]

    @staticmethod
    def rotate_vector(vector, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        print(f'vector: {vector}, axis: {axis}, theta: {math.degrees(theta)}')
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        rotation_matrix = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                                    [2 * (bc - ad), aa + cc -
                                     bb - dd, 2 * (cd + ab)],
                                    [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
        return np.dot(rotation_matrix, vector)

    def draw_body(self, color='black'):
        x_data = [vector[0] for vector in self.body]
        y_data = [vector[1] for vector in self.body]
        z_data = [vector[2] for vector in self.body]
        self.ax.plot(x_data, y_data, z_data, color=color)

    def draw_legs(self, desired_points=[(0, 10, 15), (0, 10, 15), (0, -10, 15), (0, -10, 15)], color='blue'):
        for i, leg in enumerate(self.legs):
            desired_point = desired_points[i]
            leg_vectors = []
            t.tic()
            if i < 2:
                hip_rad, shoulder_rad, wrist_rad = self.ik.from_local_xyz(
                    'r', desired_point[0], desired_point[1], desired_point[2])
                horiz_offset = -self.offsets[1]
            else:
                hip_rad, shoulder_rad, wrist_rad = self.ik.from_local_xyz(
                    'l', desired_point[0], -desired_point[1], desired_point[2])
                horiz_offset = self.offsets[1]
            t.toc()

            # changing frame of reference for drawing vectors
            wrist_rad = np.pi + wrist_rad
            shoulder_rad = shoulder_rad
            hip_rad = -hip_rad

            hip_axis = (1, 0, 0)

            # respective corner of robot
            leg_vectors.append(leg.origin)
            # z offset on hip
            leg_vectors.append(Quadruped.add_vector(leg_vectors[0],
                                                    (0, 0, -self.offsets[0])))
            # y offset on hip
            leg_vectors.append(Quadruped.add_vector(leg_vectors[-1],
                                                    (0, horiz_offset, 0)))
            shoulder_axis = (0, 1, 0)

            # upper arm
            leg_vectors.append(Quadruped.add_vector(leg_vectors[-1],
                                                    (0, 0, -self.limb_lengths[0])))
            wrist_axis = (0, 1, 0)

            # lower arm
            leg_vectors.append(Quadruped.add_vector(leg_vectors[-1],
                                                    (0, 0, -self.limb_lengths[1])))

            # apply rotations
            # wrist rotation 1
            leg_vectors[-1] = Quadruped.add_vector(leg_vectors[-2], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-1], leg_vectors[-2]), wrist_axis, wrist_rad))
            # wrist rotation 2
            leg_vectors[-1] = Quadruped.add_vector(leg_vectors[-3], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-1], leg_vectors[-3]), shoulder_axis, shoulder_rad))
            # wrist rotation 3
            leg_vectors[-1] = Quadruped.add_vector(leg_vectors[0], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-1], leg_vectors[0]), hip_axis, hip_rad))
            # shoulder rotation 1
            leg_vectors[-2] = Quadruped.add_vector(leg_vectors[-3], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-2], leg_vectors[-3]), shoulder_axis, shoulder_rad))
            # shoulder rotation 2
            leg_vectors[-2] = Quadruped.add_vector(leg_vectors[0], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-2], leg_vectors[0]), hip_axis, hip_rad))
            # hip rotation 1
            leg_vectors[-3] = Quadruped.add_vector(leg_vectors[0], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-3], leg_vectors[0]), hip_axis, hip_rad))
            # hip rotation 1
            leg_vectors[-4] = Quadruped.add_vector(leg_vectors[0], Quadruped.rotate_vector(
                Quadruped.subtract_vector(leg_vectors[-4], leg_vectors[0]), hip_axis, hip_rad))

            x_data = [vector[0] for vector in leg_vectors]
            y_data = [vector[1] for vector in leg_vectors]
            z_data = [vector[2] for vector in leg_vectors]
            self.ax.plot(x_data, y_data, z_data, color=color)


fig = plt.figure()
ax = Axes3D(fig)
ax.set_aspect("equal")
robot = Quadruped(ax)
robot.draw_body()
robot.draw_legs()

WINDOW_SIZE = 30
ax.set_xlim3d(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_ylim3d(-WINDOW_SIZE, WINDOW_SIZE)
ax.set_zlim3d(-WINDOW_SIZE, WINDOW_SIZE)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
