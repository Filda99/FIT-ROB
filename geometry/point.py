# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Point2D class to manipulate 2D points
"""

from math import sqrt


class Point2D:
    """ class to handle 2 dimensional points
    ATTRIBUTES:
        self.x : (number) the x coordinate of the point
        self.z : (number) the z coordinate of the point
    """
    x: float
    z: float

    def __init__(self, x:float=0, z:float=0):
        """ constructor of the class
        PARAMETERS:
            x : (number) the x coordinate of the point
            z : (number) the z coordinate of the point
        """
        self.x = x
        self.z = z

    def __str__(self) -> str:
        """ to be able to print a Point2D
        """
        return "(" + str(self.x) + "," + str(self.z) + ")"

    def __eq__(self, other) -> bool:
        """ to test the equality between to Point2D
        PARAMETERS:
            other : (geometry.point.Point2D) the point to compare self with
        """
        return other.x == self.x and other.z == self.z
        
    def __repr__(self) -> str:
        """ function that returns a printable representation of the self Point2D object
        """
        return f"{self}"

    @staticmethod
    def distance(p1:'Point2D', p2:'Point2D') -> float:
        """ Function that computes the euclidean distance between two points
        PARAMETERS:
            p1 : (Point2D) the first point
            p2 : (Point2D) the second point
        RETURNS:
            the distance (number)
        """
        return sqrt((p1.x - p2.x) ** 2 + (p1.z - p2.z) ** 2)
