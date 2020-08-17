"""
author: Adham Elarabawy
"""

import math

import numpy as np


class Pose:
    """
    A 2D Pose that contains x,y displacement with heading

    ...

    Attributes
    ----------
    x : double
        x position in inches
    y : double
        y position in inches
    theta :
        angle/heading in radians
    dydx :
        angle/heading in slope form

    Methods
    -------
    """

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = math.radians(theta)
        self.dydx = math.tan(math.radians(theta))

    @staticmethod
    def lerp(pose0, pose1, t):
        end_minus_start = [pose1.x - pose0.x, pose1.y - pose0.y]
        times = [element * t for element in end_minus_start]

        return [pose0.x + times[0], pose0.y + times[1]]


class QuinticSpline:
    """
    An individual quintic hermite spline

    ...

    Attributes
    ----------
    pose0 : Pose
        2D Pose for the 0th point in the spline
    pose1 : Pose
        2D Pose for the 1th point in the spline
    x_control_vector : numpy array
        vector (length 6) describing: initial x pos, initial x vel, initial x accel, final x pos, final x vel, final x accel
    y_control_vector : numpy array
        vector (length 6) describing: initial y pos, initial y vel, initial y accel, final y pos, final y vel, final y accel

    Methods
    -------
    """

    hermite_basis = np.array([[-6, 15, -10, 0, 0, 1],
                              [-3, 8, -6, 0, 1, 0],
                              [-0.5, 1.5, -1.5, 0.5, 0, 0],
                              [6, -15, 10, 0, 0, 0],
                              [-3, 7, -4, 0, 0, 0],
                              [0.5, -1, 0.5, 0, 0, 0]])

    hermite_basis_d = np.array([[0, -30, 60, -30, 0, 0],
                                [0, -15, 32, -18, 0, 1],
                                [0, -2.5, 6, -4.5, 1, 0],
                                [0, 30, -60, 30, 0, 0],
                                [0, -15, 28, -12, 0, 0],
                                [0, 2.5, -4, 1.5, 0, 0]])

    hermite_basis_dd = np.array([[0, 0, -120, 180, -60, 0],
                                 [0, 0, -60, 96, -36, 0],
                                 [0, 0, -10, 18, -9, 1],
                                 [0, 0, 120, -180, 60, 0],
                                 [0, 0, -60, 84, -24, 0],
                                 [0, 0, 10, -12, 3, 0]])

    def __init__(self, pose0, pose1, safety_scaling=1.3):
        self.pose0 = pose0
        self.pose1 = pose1
        self.safety_scaling = 1

        euclidian_distance = safety_scaling * \
            math.sqrt((pose1.x - pose0.x)**2 + (pose1.y - pose0.y)**2)

        vx0 = math.cos(pose0.theta) * euclidian_distance
        vx1 = math.cos(pose1.theta) * euclidian_distance
        ax0 = 0
        ax1 = 0

        self.x_control_vector = np.array(
            [pose0.x, vx0, ax0, pose1.x, vx1, ax1])

        vy0 = math.sin(pose0.theta) * euclidian_distance
        vy1 = math.sin(pose1.theta) * euclidian_distance
        ay0 = 0
        ay1 = 0

        self.y_control_vector = np.array(
            [pose0.y, vy0, ay0, pose1.y, vy1, ay1])

    @staticmethod
    def get_hermite_vector(t, d=0):
        """returns the hermite vector of length 6: [h0(t), h1(t), h2(t), h3(t), h4(t), h5(t)] with each element evaluated at t"""
        assert ((d >= 0) and (
            d <= 2)), "Attempted to evaluate a derivative greater than available hermite basis (or a negative derivative)"
        assert ((t >= 0) and (t <= 1)
                ), "Attempted to extrapolate out of the region of spline"
        t_vector = np.array([t**5, t**4, t**3, t**2, t, 1])
        if d == 0:
            return QuinticSpline.hermite_basis.dot(t_vector)
        if d == 1:
            return QuinticSpline.hermite_basis_d.dot(t_vector)
        if d == 2:
            return QuinticSpline.hermite_basis_dd.dot(t_vector)

    def evaluate(self, t, d=0):
        """returns the point on the trajectory by evaluating x(t) and y(t) at provided t parameter value (0<=t<=1)"""
        assert ((d >= 0) and (
            d <= 2)), "Attempted to evaluate a derivative greater than available hermite basis (or a negative derivative)"
        assert ((t >= 0) and (t <= 1)
                ), "Attempted to extrapolate out of the region of spline"
        hermite_vector = QuinticSpline.get_hermite_vector(t, d)
        return np.array([hermite_vector.dot(self.x_control_vector), hermite_vector.dot(self.y_control_vector)])

    def compute_curvature(self, t):
        return ((self.evaluate(t, 1)[0] * self.evaluate(t, 2)[1]) - (self.evaluate(t, 2)[0] * self.evaluate(t, 1)[1])) / (math.sqrt((self.evaluate(t, 1)[0]**2 + self.evaluate(t, 1)[1]**2)**3))


class Path:

    def __init__(self, waypoints):
        assert len(
            waypoints) > 1, "Path cannot be generated with only one waypoint."
        self.waypoints = waypoints
        self.num_waypoints = len(waypoints)

        self.splines = []

        for i, waypoint in enumerate(waypoints):
            if (i < self.num_waypoints - 1):
                self.splines.append(QuinticSpline(
                    waypoints[i], waypoints[i + 1]))

    def map_parameter(self, t):
        return t * (len(self.splines))

    def get_spline(self, t):
        assert ((t >= 0) and (t <= 1)), "Attempted to extrapolate out of the Path"
        normalized_t = self.map_parameter(t)
        spline_index = int(normalized_t)
        spline_local_t = normalized_t - spline_index

        if spline_index == len(self.splines):
            spline_index = len(self.splines) - 1
            spline_local_t = 1

        return self.splines[spline_index], spline_local_t

    def evaluate(self, t, d=0):
        assert ((t >= 0) and (t <= 1)), "Attempted to extrapolate out of the Path"

        spline, local_t = self.get_spline(t)
        return spline.evaluate(local_t, d)

    def compute_curvature(self, t):
        assert ((t >= 0) and (t <= 1)), "Attempted to extrapolate out of the Path"

        spline, local_t = self.get_spline(t)
        return spline.compute_curvature(local_t)

    def theta(self, t):
        """returns radians"""
        path_deriv = self.evaluate(t, 1)
        dydt = path_deriv[1]
        dxdt = path_deriv[0]
        slope = dydt / dxdt

        return math.atan(slope)

    def get_plot_values(self, d=0, resolution=100):
        t = np.linspace(0, 1, num=resolution)
        x, y = [], []
        for step in t:
            point = self.evaluate(step, d)
            x.append(point[0])
            y.append(point[1])
        return x, y

    @staticmethod
    def get_distance_between(point0, point1):
        return math.sqrt((point0[0] - point1[0])**2 + (point0[1] - point1[1])**2)

    @staticmethod
    def transform(pose0, pose1):
        initial_translation = [pose0.x, pose0.y]
        last_translation = [pose1.x, pose1.y]
        initial_rotation = [math.cos(pose0.theta), math.sin(pose0.theta)]
        last_rotation = [math.cos(pose1.theta), math.sin(pose1.theta)]

        initial_unary = [math.cos(math.radians(-math.degrees(pose0.theta))),
                         math.sin(math.radians(-math.degrees(pose0.theta)))]

        matrix0 = [last_translation[0] - initial_translation[0],
                   last_translation[1] - initial_translation[1]]

        m_translation = [matrix0[0] * initial_unary[0] - matrix0[1] * initial_unary[1],
                         matrix0[0] * initial_unary[1] + matrix0[1] * initial_unary[0]]
        m_rotation = [last_rotation[0] * initial_unary[0] - last_rotation[1] * initial_unary[1],
                      last_rotation[1] * initial_unary[0] + last_rotation[0] * initial_unary[1]]

        # normalize rotation matrix
        magnitude = math.sqrt(m_rotation[0]**2 + m_rotation[1]**2)
        if magnitude > 10**-9:
            m_rotation[0] /= magnitude
            m_rotation[1] /= magnitude
        else:
            m_rotation[0] = 1
            m_rotation[1] = 0

        return m_translation, m_rotation

    @staticmethod
    def twistify(pose0, pose1):
        transform_translation, transform_rotation = Path.transform(
            pose0, pose1)
        dtheta = math.atan2(transform_rotation[1], transform_rotation[0])

        half_dtheta = dtheta / 2
        cos_minus_one = transform_rotation[0] - 1

        if (abs(cos_minus_one) < 10**-9):
            half_theta_by_tan_of_half_dtheta = 1 - 1 / 12 * dtheta * dtheta
        else:
            half_theta_by_tan_of_half_dtheta = - \
                (half_dtheta * transform_rotation[1]) / cos_minus_one

        # rotation

        rotate_by = [half_theta_by_tan_of_half_dtheta, -half_dtheta]
        times_by = math.sqrt(
            half_theta_by_tan_of_half_dtheta**2 + half_dtheta**2)

        rotated = [transform_translation[0] * rotate_by[0] - transform_translation[1] * rotate_by[1],
                   transform_translation[0] * rotate_by[1] + transform_translation[1] * rotate_by[0]]
        final = [rotated[0] * times_by, rotated[1] * times_by]

        return final[0], final[1], dtheta
