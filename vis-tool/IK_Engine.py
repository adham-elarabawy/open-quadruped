import math

import numpy as np

DEBUG = False


class Leg:
    def __init__(self, origin):
        self.init_origin = origin
        self.origin = origin
        self.hip_rad = None
        self.shoulder_rad = None
        self.wrist_rad = None
        # local
        self.x = None
        self.y = None
        self.z = None
        # global
        self.g_x = None
        self.g_y = None
        self.g_z = None

    def __str__(self):
        return f'origin: {self.origin}, angles: [hip: {math.degrees(self.hip_rad)}, shoulder: {math.degrees(self.shoulder_rad)}, wrist: {math.degrees(self.wrist_rad)}], endpoint: {(self.x, self.y, self.z)}'


class InverseKinematics:

    def __init__(self, forearm, shoulder, body_dim, hip_offset):
        '''
        body_dim: (length, width,thickness) in mm
        '''
        self.wrist = forearm
        self.shoulder = shoulder
        self.body_dim = body_dim
        self.hip_offset = hip_offset  # z, y
        self.joint_angles = []

    def local_translation_engine(self, legs_xyz):
        '''
        Translation engine for
        '''
        try:
            joint_angles = []
            for i, (x, y, z) in enumerate(legs_xyz):
                h1 = math.sqrt(self.hip_offset[0]**2 + self.hip_offset[1]**2)
                h2 = math.sqrt(z**2 + y**2)
                alpha_0 = math.atan(y / z)
                alpha_1 = math.atan(self.hip_offset[1] / self.hip_offset[0])
                alpha_2 = math.atan(self.hip_offset[0] / self.hip_offset[1])
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
                    (h**2 + self.shoulder**2 - self.wrist**2) / (2 * h * self.shoulder)) - phi
                theta_w = math.acos((self.wrist**2 + self.shoulder **
                                     2 - h**2) / (2 * self.wrist * self.shoulder))

                if i < 2:
                    joint_angles.append((theta_h, theta_s, theta_w))
                else:
                    joint_angles.append((-theta_h, theta_s, theta_w))
            self.joint_angles = joint_angles
        except:
            print("Out of bounds.")
        return self.joint_angles


class Quadruped:

    def __init__(self, ax=0, origin=(0, 0, 100), body_dim=(230, 78), limb_lengths=(107, 130), offsets=(10, 60), height=170):
        '''
        body_dim: (length, width,thickness) in mm
        limb_lengths: (upper_arm, bottom_arm) in mm
        offsets: (z_offset, y_offset) in mm
        '''
        self.ax = ax
        self.body_dim = body_dim
        self.limb_lengths = limb_lengths
        self.offsets = offsets
        self.init_origin = origin
        self.origin = origin
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.height = height

        self.ik = InverseKinematics(
            limb_lengths[1], limb_lengths[0], self.body_dim, self.offsets)

        self.body = [(origin[0] - self.body_dim[0] / 2,
                      origin[1] - self.body_dim[1] / 2, origin[2]),
                     (origin[0] + self.body_dim[0] / 2,
                      origin[1] - self.body_dim[1] / 2, origin[2]),
                     (origin[0] + self.body_dim[0] / 2,
                      origin[1] + self.body_dim[1] / 2, origin[2]),
                     (origin[0] - self.body_dim[0] / 2,
                      origin[1] + self.body_dim[1] / 2, origin[2]),
                     (origin[0] - self.body_dim[0] / 2,
                      origin[1] - self.body_dim[1] / 2, origin[2])]

        # back_right_leg, front_right_leg, front_left_leg, back_left_leg
        self.legs = [Leg((-self.body_dim[0] / 2, -self.body_dim[1] / 2, origin[2])),
                     Leg((self.body_dim[0] / 2, -
                          self.body_dim[1] / 2, origin[2])),
                     Leg((self.body_dim[0] / 2,
                          self.body_dim[1] / 2, origin[2])),
                     Leg((-self.body_dim[0] / 2, self.body_dim[1] / 2, origin[2]))]

    @staticmethod
    def add_vector(base_vector, increment):
        '''
        Adds the given vectors element-wise in the order: base_vector + increment
        '''
        assert(len(base_vector) == len(increment))
        return [val + increment[i] for i, val in enumerate(base_vector)]

    @staticmethod
    def subtract_vector(base_vector, increment):
        '''
        Subtracts the given vectors element-wise in the order: base_vector - increment
        '''
        assert(len(base_vector) == len(increment))
        return [val - increment[i] for i, val in enumerate(base_vector)]

    @staticmethod
    def rotate_vector(vector, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        global DEBUG
        if DEBUG:
            print(
                f'vector: {vector}, axis: {axis}, theta: {math.degrees(theta)}')
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

    def draw_legs(self, color='blue'):
        for i, leg in enumerate(self.legs):
            leg_vectors = []

            # IMPORTANT
            # changing frame of reference for drawing vectors
            wrist_rad = np.pi + leg.wrist_rad
            shoulder_rad = leg.shoulder_rad
            hip_rad = -leg.hip_rad

            if i < 2:
                horiz_offset = -self.offsets[1]
            else:
                horiz_offset = self.offsets[1]

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

            for i, vector in enumerate(leg_vectors):
                leg_vectors[i] = Quadruped.rotate_vector(
                    vector, [0, 0, 1], -self.yaw)
            for i, vector in enumerate(leg_vectors):
                leg_vectors[i] = Quadruped.rotate_vector(
                    vector, [0, 1, 0], -self.pitch)
            for i, vector in enumerate(leg_vectors):
                leg_vectors[i] = Quadruped.rotate_vector(
                    vector, [1, 0, 0], -self.roll)

            x_data = [vector[0] for vector in leg_vectors]
            y_data = [vector[1] for vector in leg_vectors]
            z_data = [vector[2] for vector in leg_vectors]
            self.ax.plot(x_data, y_data, z_data, color=color)

    def fully_define(self, leg_points):
        global DEBUG
        for i, leg in enumerate(self.legs):
            leg.x = leg_points[i][0]
            leg.y = leg_points[i][1]
            leg.z = leg_points[i][2]

        leg_angle_sets = self.ik.local_translation_engine(leg_points)

        for i, leg_angle_set in enumerate(leg_angle_sets):
            self.legs[i].hip_rad = leg_angle_set[0]
            self.legs[i].shoulder_rad = leg_angle_set[1]
            self.legs[i].wrist_rad = leg_angle_set[2]

    def start_position(self):
        for leg in self.legs:
            leg.origin = leg.init_origin
        self.origin = self.init_origin
        starting_points = [(50, 80, self.height),
                           (-50, 80, self.height),
                           (-50, 80, self.height),
                           (50, 80, self.height)]
        self.fully_define(starting_points)
        if DEBUG:
            for leg in self.legs:
                print(leg)

    def get_points_from_buffer(self):
        return [(leg.x, leg.y, leg.z) for leg in self.legs]

    def shift_body_xyz(self, x, y, z):
        local_x_shift = x
        local_y_shift = y
        local_z_shift = z
        shifts = (local_x_shift, local_y_shift, local_z_shift)
        for i, leg in enumerate(self.legs):
            if i == 1 or i == 2:
                leg.x += -local_x_shift
            else:
                leg.x += local_x_shift
            if i < 2:
                leg.y += local_y_shift
            else:
                leg.y += -local_y_shift
            leg.z += local_z_shift
        self.fully_define(self.get_points_from_buffer())
        self.origin = (self.origin[0] + local_x_shift,
                       self.origin[1] + local_y_shift,
                       self.origin[2] + local_z_shift)

        self.body = [(self.origin[0] - self.body_dim[0] / 2, self.origin[1] - self.body_dim[1] / 2, self.origin[2]),
                     (self.origin[0] + self.body_dim[0] / 2,
                      self.origin[1] - self.body_dim[1] / 2, self.origin[2]),
                     (self.origin[0] + self.body_dim[0] / 2,
                      self.origin[1] + self.body_dim[1] / 2, self.origin[2]),
                     (self.origin[0] - self.body_dim[0] / 2,
                      self.origin[1] + self.body_dim[1] / 2, self.origin[2]),
                     (self.origin[0] - self.body_dim[0] / 2, self.origin[1] - self.body_dim[1] / 2, self.origin[2])]
        for leg in self.legs:
            leg.origin = [leg.origin[i] + shift for i,
                          shift in enumerate(shifts)]

    def shift_body_rotation(self, yaw, pitch, roll):
        try:
            # YAW CALCULATIONS
            self.yaw = yaw
            self.pitch = pitch
            self.roll = roll
            # YAW

            for i, leg in enumerate(self.legs):
                # Front Right Leg
                if i == 1:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(x_g / y_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 + yaw
                    x_g = radius * math.sin(alpha_1)
                    y_g = radius * math.cos(alpha_1)
                    leg.x = x_g - (self.body_dim[0] / 2 + self.init_origin[0])
                    leg.y = y_g - (self.body_dim[1] / 2 + self.init_origin[1])
                # Front Left Leg
                if i == 2:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(x_g / y_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 - yaw
                    x_g = radius * math.sin(alpha_1)
                    y_g = radius * math.cos(alpha_1)
                    leg.x = x_g - (self.body_dim[0] / 2 + self.init_origin[0])
                    leg.y = y_g - (self.body_dim[1] / 2 + self.init_origin[1])
                # Back Right Leg
                if i == 0:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(y_g / x_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 + yaw
                    x_g = radius * math.cos(alpha_1)
                    y_g = radius * math.sin(alpha_1)
                    leg.x = -x_g + (self.init_origin[0] + self.body_dim[0] / 2)
                    leg.y = y_g - (self.init_origin[1] + self.body_dim[1] / 2)
                # Back Left Leg
                if i == 3:
                    x_g = self.init_origin[0] + self.body_dim[0] / 2 + leg.x
                    y_g = self.init_origin[1] + self.body_dim[1] / 2 + leg.y
                    alpha_0 = math.atan(y_g / x_g)
                    radius = math.sqrt(x_g**2 + y_g**2)
                    alpha_1 = alpha_0 - yaw
                    x_g = radius * math.cos(alpha_1)
                    y_g = radius * math.sin(alpha_1)
                    leg.x = -x_g + (self.init_origin[0] + self.body_dim[0] / 2)
                    leg.y = y_g - (self.init_origin[1] + self.body_dim[1] / 2)

            # PITCH CALCULATIONS
            sig_z = sum([leg.z for leg in self.legs]) / 4
            z_i = self.body_dim[0] / 2 * math.sin(pitch)
            x_i = z_i / math.tan((math.pi - pitch) / 2)
            for i, leg in enumerate(self.legs):
                if i == 1 or i == 2:  # front
                    self.legs[i].z = sig_z + z_i
                    self.legs[i].x = self.legs[i].x - x_i
                if i == 0 or i == 3:  # back
                    self.legs[i].z = sig_z - z_i
                    self.legs[i].x = self.legs[i].x - x_i

            # ROLL CALCULATIONS
            sig_z_front = (self.legs[1].z + self.legs[2].z) / 2
            sig_z_back = (self.legs[0].z + self.legs[3].z) / 2
            z_i = self.body_dim[1] / 2 * math.sin(roll)
            y_i = z_i / math.tan((math.pi - roll) / 2)
            for i, leg in enumerate(self.legs):
                if i == 0:
                    self.legs[i].z = sig_z_back + z_i
                    leg.y -= y_i
                if i == 1:
                    self.legs[i].z = sig_z_front + z_i
                    leg.y -= y_i
                if i == 2:
                    self.legs[i].z = sig_z_front - z_i
                    leg.y += y_i
                if i == 3:
                    self.legs[i].z = sig_z_back - z_i
                    leg.y += y_i
        except:
            print("Out of bounds.")
        self.fully_define(self.get_points_from_buffer())

        for i, vector in enumerate(self.body):
            self.body[i] = Quadruped.rotate_vector(
                vector, [0, 0, 1], -self.yaw)
        for i, vector in enumerate(self.body):
            self.body[i] = Quadruped.rotate_vector(
                vector, [0, 1, 0], -self.pitch)
        for i, vector in enumerate(self.body):
            self.body[i] = Quadruped.rotate_vector(
                vector, [1, 0, 0], -self.roll)
