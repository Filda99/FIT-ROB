# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"

"""
    This file provides a Simplex class used by the Nelder and Mead algorithm
"""

from typing import List  # for python 3.8 compatibility

from tp_icp.vertex import Vertex
from robot.pose import Pose3D
from geometry.point import Point2D
from tp_icp.association import Association


class Simplex:
    """ Class that handles a simplex
        The dimension (3) is hard coded
        Thus it has 4 vertices (see Vertex class)
        ATTRIBUTES:
            self.dimension: (integer strictly positive) the dimension of the simplex, value = 3 (hardcoded)
            self.vertices: (list of Vertice) the vertices of the simplex (list)
    """

    dimension: int
    vertices: List[Vertex]

    def __init__(self):
        """ Constructor of the class
            Note that the vertices can be accessed with the [] operator
            simplex[0] means the first vertice
        """
        self.dimension = 3
        self.vertices = []  # the vertices of the simplex
        # the simplex should have N+1 vertices (N being the dimension of the simplex)
        for _ in range(self.dimension + 1):
            self.vertices.append(Vertex())

    def sort_vertices(self):
        """ Function that sorts all the vertices of the simplex from the lowest cost to highest
            It updates the vertices attribute, returns nothing
        """
        # TODO
        pass

    def update_cost(self, model:List[Point2D], scene:List[Point2D], association:List[Association]):
        """ Function that updates the cost of all the vertices in the vertices list
            It loops over the vertices and update each cost
            It returns nothing
            :param model: (list of geometry.point.Point2D) the points of the model
            :param scene: (list of geometry.point.Point2D) the points of the scene
            :param association: (list of tp_icp.association.Association) the model/scene point associations
        """
        # TODO
        pass

    def get_centroid(self):
        """ Function that returns the centroid of the current simplex
            The centroid is center of mass of the simplex (average of all the vertices except the worst one)
            It is assumed that the simplex is ordered before calling this function
            :return: (tp_icp.vertex.Vertex) the centroid
        """
        centroid = Vertex(Pose3D(0, 0, 0))
        # TODO
        return centroid

    def __setitem__(self, key:int, value:float):
        """ Function to override the [] operator when doing an affectation
            this allows to do: my_simplex[key] = value
            Parameters:
                key: (int) the index of the vertice
                value: (number) the value for the vertice
        """
        if not isinstance(key, int):
            # if the key is not an integer
            raise IndexError("Key should be an integer")
        if key > self.dimension:
            # if the key is out of range
            raise IndexError("Out of range")

        if not isinstance(value, Vertex):
            # if the value is not a Vertice
            raise ValueError("value must be a vertice")

        self.vertices[key] = value

    def __getitem__(self, key:int):
        """ Function to override the [] operator when doing a get
            this allows to do: var = my_simplex[key]
            Parameters:
                key: (int) the index of the vertice
        """
        if not isinstance(key, int):
            # if the key is not a number
            raise IndexError("Key should be an integer")
        if key > self.dimension:
            # if the key is out of range
            raise IndexError("Out of range")

        return self.vertices[key]

    def __str__(self) -> str:
        """ Function to convert a simplex into a string
            It lists the vertices of the simplex and their position
        """
        string = ""
        for i, vertice in enumerate(self.vertices):
            string += f"Simplex[{i}] : {vertice}\n"
        return string
