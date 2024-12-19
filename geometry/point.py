"""
    Project: ROBa project
    File: point.py
    Description: This file contains the Point class and its associated methods for representing points in the environment.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""

from math import sqrt


class Point2D:
    """ 
        Class to handle 2 dimensional points
        Attributes:
            self.x : (number) the x coordinate of the point
            self.z : (number) the z coordinate of the point
    """
    x: float
    y: float

    def __init__(self, x:float=0, y:float=0):
        """ 
            Constructor of the class
            Parameters:
                x : (number) the x coordinate of the point
                y : (number) the y coordinate of the point
        """
        self.x = x
        self.y = y

    @staticmethod
    def distance(p1:'Point2D', p2:'Point2D') -> float:
        """ Method that computes the euclidean distance between two points
        Parameters:
            p1 : (Point2D) the first point
            p2 : (Point2D) the second point
        Returns:
            the distance (number)
        """
        return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
