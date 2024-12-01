# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Node class used by the A* algorithm
"""

from geometry.point import Point2D


class Node:
    """ class to handle a node in the A* algorithm
    ATTRIBUTES:
        self.walkable: (boolean) to define if the node corresponds to an obstacle or a free space
        self.position: (geometry.point.Point2D) the coordinates of the node
        self.parent_position: (geometry.point.Point2D) the position of the parent node
        self.s_cost: (number) the cost from the starting node
        self.h_cost: (number) the cost to the target node (heuristic)
        self.f_cost: (number) the total cost of the node
        self.is_opened: (boolean) defines if the node is in the opened list or not
        self.is_closed: (boolean) defines if the node is in the closed list or not
        self.is_path: (boolean) defines if the node belongs to the computed path or not
    """
    walkable: bool
    position: Point2D
    parent_position: Point2D
    s_cost: float
    h_cost: float
    f_cost: float
    is_opened: bool
    is_closed: bool
    is_path: bool

    def __init__(self, walkable:bool=False, position:Point2D=None, s_cost:float=0, h_cost:float=0, parent_position:Point2D=None):
        """ constructor of the class
        PARAMETERS:
            walkable: (boolean) to define if the node corresponds to an obstacle or a free space
            position: (geometry.point.Point2D) the coordinates of the node
            s_cost: (number) the cost from the starting node
            h_cost: (number) the cost to the target node (heuristic)
            self.parent_position: (geometry.point.Point2D) the position of the parent node
        """
        if position is None:
            position = Point2D()
        if parent_position is None:
            parent_position = Point2D()

        self.walkable = walkable                # to define if the node correspond to an obstacle or a free space
        self.position = position                # a Point2D variable that corresponds to the coordinates of the node
        self.parent_position = parent_position  # the position of the parent node (Point2D)
        self.s_cost = s_cost                    # the cost from the starting node
        self.h_cost = h_cost                    # the cost to the target node (heuristic)
        self.f_cost = s_cost + h_cost           # the total cost of the node
        self.is_opened = False                  # defines if the node is in the opened list or not
        self.is_closed = False                  # defines if the node is in the closed list or not
        self.is_path = False                    # defines if the node belongs to the computed path or not

    def __lt__(self, other):
        """ the "lower than" operator, to be able to compare the nodes between each other
        PARAMETERS:
            other: (tp_a_star.node.Node) the node to be compared with
        RETURN:
            True or False
        """
        # if the total costs are equivalent, the lowest is the one with the lowest cost to the starting node
        # otherwise the lowest if the one with the lowest cost
        # TODO
        return True

    def update_costs(self, s_cost:float, h_cost:float, parent_position:Point2D):
        """ function to update a cost according to a parent node
        PARAMETERS:
            s_cost: (number) the cost from the starting node
            h_cost: (number) the cost to the target node (heuristic)
            parent_position: (geometry.point.Point2D) the position of the parent node
        """
        # TODO
        pass

    def __str__(self) -> str:
        return f"|{self.walkable}, {self.position}, {self.parent_position}|"
