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
    """ 
        Class to handle a 3D pose (2D position and 1 orientation)
        Attributes:
            self.x: (number in m) the x position
            self.y: (number in m) the y position
            self.theta: (number in rad) the orientation
    """
    x: float
    y: float
    theta: float

    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        """ 
            Constructor of the class
            Parameters:
                x: (number in m) the x position
                y: (number in m) the y position
                theta: (number in rad) the orientation
        """
        self.x = x          # m      x position of the robot
        self.y = y          # m      z position of the robot
        self.theta = theta  # rad    orientation of the robot