import numpy as np
from scipy.spatial.transform import Rotation as R

class BodyIKModel:
    """Quadruped Body Inverse Kinematics Model
     - Uses transformation matrices to rotate the body about each axis for euler angle manipulation.

     args: width (mm), length(mm), height (mm)
     defs: width of the body in mm, length of the body in mm, desired height of body initially.
     
     methods: transform, get_htf_vectors
     """
    def reset_pose(self):

        # only doing this for sake of concision
        length = self.length
        width = self.width
        height = self.height

        self.body_points = np.array([[length/2, -width/2, height],
                                     [length/2, width/2, height],
                                     [-length/2, -width/2, height], 
                                     [-length/2, width/2, height]])
        
        self.leg_points =  np.array([[length/2, -width/2, 0],
                                     [length/2, width/2, 0],
                                     [-length/2, -width/2, 0], 
                                     [-length/2, width/2, 0]])
    def __init__(self, width, length, height):
        self.width = width
        self.length = length
        self.height = height

        self.reset_pose()
        
    def transform(self, yaw, pitch, roll):
        # yaw rotation
        rot_axis = np.array([0,0,1])
        rot_vector = yaw * rot_axis
        rotation = R.from_rotvec(rot_vector)
        for i, point in enumerate(self.body_points):
            self.body_points[i] = rotation.apply(point)
        
        # pitch rotation
        rot_axis = np.array([0,1,0])
        rot_vector = pitch * rot_axis
        rotation = R.from_rotvec(rot_vector)
        for i, point in enumerate(self.body_points):
            self.body_points[i] = rotation.apply(point)
        
        # roll rotation
        rot_axis = np.array([1,0,0])
        rot_vector = roll * rot_axis
        rotation = R.from_rotvec(rot_vector)
        for i, point in enumerate(self.body_points):
            self.body_points[i] = rotation.apply(point)

    def get_htf_vectors(self):
        return np.subtract(self.body_points, self.leg_points)
