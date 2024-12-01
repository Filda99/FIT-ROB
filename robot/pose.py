# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
 This file provides a Pose class
"""


class Pose3D:
    """ A class to handle a 3D pose (2D position and 1 orientation)
        ATTRIBUTES:
            self.x: (number in m) the x position
            self.z: (number in m) the z position
            self.theta: (number in rad) the orientation
    """
    x: float
    z: float
    theta: float

    def __init__(self, x=0, z=0, theta=0):
        """ Constructor of the class
        PARAMETERS:
            x: (number in m) the x position
            z: (number in m) the z position
            theta: (number in rad) the orientation
        """
        self.x = x          # m      x position of the robot
        self.z = z          # m      z position of the robot
        self.theta = theta  # rad    orientation of the robot

    def __str__(self):
        """ to print a pose 3D
        """
        return f"({self.x}, {self.z}, {self.theta})"

    def __eq__(self, other):
        """
        Overwrite the == operator
        :param other: (Pose3D) the pose to be compared with
        """
        return self.x == other.x and self.z == other.z and self.theta == other.theta

    def __add__(self, other):
        """
        Overwrite the self + other operator
        :param other: (Pose3D) the pose to add
        """
        if isinstance(other, Pose3D):
            return Pose3D(self.x + other.x, self.z + other.z, self.theta + other.theta)
        else:
            return Pose3D(self.x + other, self.z + other, self.theta + other)

    def __radd__(self, other):
        """
        Overwrite the other + self operator
        :param other: (Pose3D) the pose to add
        """
        return self + other

    def __sub__(self, other):
        """
        Overwrite the self - other operator
        :param other: (Pose3D) the pose to sub
        """
        if isinstance(other, Pose3D):
            return Pose3D(self.x - other.x, self.z - other.z, self.theta - other.theta)
        else:
            return Pose3D(self.x - other, self.z - other, self.theta - other)

    def __rsub__(self, other):
        """
        Overwrite the other + self operator
        :param other: (Pose3D) the pose to add
        """
        return -self + other

    def __neg__(self):
        """
        Overwrite the negative operator (-self)
        """
        return Pose3D(-self.x, -self.z, -self.theta)

    def __mul__(self, other):
        """
        Overwrite the self * other operator
        :param other: (Pose3D) the pose to add
        """
        if isinstance(other, Pose3D):
            return Pose3D(self.x * other.x, self.z * other.z, self.theta * other.theta)
        else:
            return Pose3D(self.x * other, self.z * other, self.theta * other)

    def __rmul__(self, other):
        """
        Overwrite the other * self operator
        :param other: (Pose3D) the pose to add
        """
        return self * other
