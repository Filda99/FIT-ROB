"""
    Project: ROBa project
    File: pose.py
    Description: This file contains the Pose3D class and its associated methods for representing 3D poses in the robotics project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""


class Pose3D:
    """ A class to handle a 3D pose (2D position and 1 orientation)
        Attributes:
            self.x: (number in m) the x position
            self.y: (number in m) the y position
            self.theta: (number in rad) the orientation
    """
    x: float
    y: float
    theta: float

    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        """ Constructor of the class
        Parameters:
            x: (number in m) the x position
            y: (number in m) the y position
            theta: (number in rad) the orientation
        """
        self.x = x          # m      x position of the robot
        self.y = y          # m      z position of the robot
        self.theta = theta  # rad    orientation of the robot

    def __str__(self):
        """ to print a pose 3D
        """
        return f"({self.x}, {self.y}, {self.theta})"

    def __eq__(self, other):
        """
        Overwrite the == operator
        :param other: (Pose3D) the pose to be compared with
        """
        return self.x == other.x and self.y == other.z and self.theta == other.theta

    def __add__(self, other):
        """
        Overwrite the self + other operator
        :param other: (Pose3D) the pose to add
        """
        if isinstance(other, Pose3D):
            return Pose3D(self.x + other.x, self.y + other.y, self.theta + other.theta)
        else:
            return Pose3D(self.x + other, self.y + other, self.theta + other)

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
            return Pose3D(self.x - other.x, self.y - other.y, self.theta - other.theta)
        else:
            return Pose3D(self.x - other, self.y - other, self.theta - other)

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
        return Pose3D(-self.x, -self.y, -self.theta)

    def __mul__(self, other):
        """
        Overwrite the self * other operator
        :param other: (Pose3D) the pose to add
        """
        if isinstance(other, Pose3D):
            return Pose3D(self.x * other.x, self.y * other.y, self.theta * other.theta)
        else:
            return Pose3D(self.x * other, self.y * other, self.theta * other)

    def __rmul__(self, other):
        """
        Overwrite the other * self operator
        :param other: (Pose3D) the pose to add
        """
        return self * other
