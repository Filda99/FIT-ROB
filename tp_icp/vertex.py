# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"

"""
    This file provides a Vertex Class for the simplex (used by the Nelder and Mead algorithm)
"""
from typing import List  # for python 3.8 compatibility
from math import cos, sin

from geometry.point import Point2D
from robot.pose import Pose3D
from tp_icp.association import Association
import copy as copy


class Vertex:
    """ Class that handles a vertex of the simplex (cf simplex.py)
    """
    pose: Pose3D
    cost: float

    def __init__(self, pose:Pose3D=None, cost:float=0):
        """ Constructor of the class
            Attributes:
                self.pose: (Pose3D) the pose (x, z, theta) of the vertice
                self.cost: (number) the cost value of the vertice
        """
        if pose is None:
            pose = Pose3D()
        self.pose = copy.copy(pose)
        self.cost = cost

    def update_cost(self, model:List[Point2D], scene:List[Point2D], associations:List[Association]):
        """ Function that updates the cost of the vertex according to a model and a scene
            The cost corresponds to the euclidean distance between the model and the new scene after doing the scene
            transformation (reminder: a vertex corresponds to a transformation, two translations x,z
            and a rotation theta)
            Parameters:
                model: (list of geometry.point.Point2D) the points of the model
                scene: (list of geometry.point.Point2D) the points of the scene
                associations: (list of tp_icp.association.Association)
            The cost is computed by adding the euclidean distance of each pair of points model/scene
            It returns nothing
        """
        cost = 0
        # TODO
        self.cost = cost

    def __lt__(self, other):
        """ Function to override the < operator
            Parameters:
                other: (Vertice) the Vertice to compare with
        """
        if not isinstance(other, Vertex):
            # if other is not a Vertice
            raise ValueError("A Vertice can only be compared to an other Vertice")
        return self.cost < other.cost

    def __le__(self, other):
        """ Function to override the <= operator
            Parameters:
                other: (Vertice) the Vertice to compare with
        """
        if not isinstance(other, Vertex):
            # if other is not a Vertice
            raise ValueError("A Vertice can only be compared to an other Vertice")
        return self.cost <= other.cost

    def __eq__(self, other):
        """ Function to override the == operator
            Parameters:
                other: (Vertice) the Vertice to compare with
        """
        if not isinstance(other, Vertex):
            # if other is not a Vertice
            raise ValueError("A Vertice can only be compared to an other Vertice")
        return self.cost == other.cost

    def __gt__(self, other):
        """ Function to override the > operator
            Parameters:
                other: (Vertice) the Vertice to compare with
        """
        if not isinstance(other, Vertex):
            # if other is not a Vertice
            raise ValueError("A Vertice can only be compared to an other Vertice")
        return self.cost > other.cost

    def __ge__(self, other):
        """ Function to override the >= operator
            Parameters:
                other: (Vertice) the Vertice to compare with
        """
        if not isinstance(other, Vertex):
            # if other is not a Vertice
            raise ValueError("A Vertice can only be compared to an other Vertice")
        return self.cost >= other.cost

    def __str__(self):
        """ Function to convert a Vertice into a string
            The sting is of the form : cost | x, z, theta
        """
        string = f"{self.cost} | {self.pose}"
        return string

    def __add__(self, other):
        """
        Overwrite the self + other operator
        :param other: (Vertex) the vertex to add
        """
        if isinstance(other, Vertex):
            return Vertex(self.pose + other.pose)
        else:
            return Vertex(self.pose + other)

    def __radd__(self, other):
        """
        Overwrite the other + self operator
        :param other: (Vertex) the vertex to add
        """
        return self + other

    def __sub__(self, other):
        """
        Overwrite the self - other operator
        :param other: (Vertex) the vertex to sub
        """
        if isinstance(other, Vertex):
            return Vertex(self.pose - other.pose)
        else:
            return Vertex(self.pose - other)

    def __rsub__(self, other):
        """
        Overwrite the other + self operator
        :param other: (Vertex) the vertex to sub
        """
        return -self + other

    def __neg__(self):
        """
        Overwrite the negative operator (-self)
        """
        return Vertex(-self.pose)

    def __mul__(self, other):
        """
        Overwrite the self * other operator
        :param other: (Vertex) the vertex to multiply
        """
        if isinstance(other, Vertex):
            return Vertex(self.pose * other.pose)
        else:
            return Vertex(self.pose * other)

    def __rmul__(self, other):
        """
        Overwrite the other * self operator
        :param other: (Vertex) the vertex to be multiplied by
        """
        return self * other
