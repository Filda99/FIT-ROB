# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Node class and a RRT_Star class
"""
from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D
from geometry.segment import Segment2D
from math import sqrt, atan2, cos, sin
from environment.seg_map import SegMap
import random as random

def dist2D(p1:Point2D, p2:Point2D) -> float:
    """ Function that returns the euclidean distance between p1 and p2.
    p1 and p2 can be Node or Point2D. p1 and p2 must have x and z coordinates.
    PARAMETERS:
        p1: (Node/Point2D) first node/point
        p2: (Node/Point2D) second node/point
    RETURNS:
        the distance (number)
    """
    return sqrt((p1.x-p2.x)**2 + (p1.z-p2.z)**2)

class Node:
    """ class to handle Nodes in the RRT
    ATTRIBUTES:
        self.x: (number in m) x position of the node in the world frame
        self.z: (number in m) z position of the node in the world frame
        self.cost: (number) cost of the node (distance from this node to the root node)
        self.parent: (int) id of the parent node in the tree
        self.id: (int) id of the node, correspond to the index of the node in the tree list
    """

    def __init__(self, x_coord:float, z_coord:float):
        """ constructor of the class
            PARAMETERS:
                x_coord: (number in m) x position of the node in the world frame
                z_coord: (number in m) z position of the node in the world frame
        """
        self.x = x_coord     # value of the node
        self.z = z_coord     # value of the node
        self.cost = None     # cost of the node (distance from this node to the initial one)
        self.parent = None   # direct parent of the node in the graph
        self.id = None       # id of the node

    @staticmethod
    def step_from_to(n1:'Node', n2:'Node', epsilon:float) -> 'Node':
        """ static function that returns a new node at an epsilon distance of
            the node n1 in the direction of the node n2
        PARAMETERS:
            n1: (Node) first node (the "from" node)
            n2: (Node) second node (the "to" node)
            epsilon: (number in m) the maximal distance of the new node to the "from" node
        RETURNS:
            the new node (Node)
        """
        # TODO
        pass

    def __str__(self):
        """ function to print a node
        """
        return f"[ ({self.x},{self.z}), {self.cost}, {self.parent} ]"

    def __eq__(self, other):
        """ function to test is two nodes are equals (only position) """
        return self.x == other.x and self.z == other.z

    def __neq__(self, other):
        """ function to test is two nodes are equals (only position) """
        return not (self == other)


class RRTStar:
    """ class to handle Rapidly Random Tree
    ATTRIBUTES:
        self.nodes: (list of Node) nodes of the graph
        self.path: (list of Node) path to go to the goal node from the start node
        self.epsilon: (number in m) distance step for the creation of new node
        self.start: (geometry.point.Point2D) the starting point2D for the graph
        self.goal: (geometry.point.Point2D) the point2D goal
        self.succeed: (bool) to indicates if the tree has been built correctly or not
    """
    nodes: List[Node]
    path: List[Node]
    start: Point2D
    goal: Point2D
    succeed: bool
    epsilon: float
    max_nodes: int
    max_distance: float
    rand_nodes_before_target: int

    def __init__(self):
        """ constructor of the class
        """
        self.nodes = []       # nodes of the graph
        self.path = []        # path to go to the goal node from the start node

        self.start = None     # the starting point2D for the graph
        self.goal = None      # the point2D goal
        self.succeed = False  # to indicates if the tree has been built correctly or not

        self.epsilon = 0.5                  # distance step to generate a new node in the graph
        self.max_nodes = 1000               # maximal number of nodes for the tree
        self.radius = 0.3                   # radius inside which we are looking for a better parent
        self.max_distance = 0.2             # maximal distance to consider that we have reached the target
        self.rand_nodes_before_target = 50  # number of random nodes before considering the target as random node

    def choose_parent(self, my_map:SegMap, nearest:Node, node:Node) -> Node :
        """ function to choose a parent to the node "node" according to the map
        PARAMETERS:
            node: (Node) the node we want to find the best parent
            my_map: (environment.seg_map.SegMap) the map we are creating the graph in
            nearest: (Node) the nearest node according to the euclidean distance
        RETURN:
            the updated node "node"
        """
        # TODO
        return node

    @staticmethod
    def trajectory_free(my_map:SegMap, start:Node, goal:Node) -> bool:
        """ function to test if the trajectory from the node start to the node goal
            does not intersect any obstacle in the map
            PARAMETERS:
                my_map: (environment.seg_map.SegMap) the known map
                start: (Node) the starting node
                goal: (Node) the target node
            RETURN:
                True or False
        """
        # TODO
        return True

    @staticmethod
    def get_rand_node(my_map:SegMap) -> Node:
        """ function that returns a random node in the map
        PARAMETERS:
            my_map: (environment.seg_map.SegMap) the known map
        RETURN:
            a random node (Node)
        """
        # TODO
        return Node(0, 0)

    def build_rrt(self, my_map:SegMap, start:Point2D, goal:Point2D):
        """ function that build an RRT, according to a start position, a goal position and
            a map (set of segments)
            PARAMETERS:
                my_map: (environment.seg_map.SegMap) the known map
                start: (geometry.point.Point2D) the starting point for the tree
                goal: (geometry.point.Point2D) the target point (position)
        """
        self.start = start      # the starting point2D for the graph
        self.goal = goal        # the point2D goal
        self.path = []          # the set of nodes to go from the start to the goal
        self.succeed = False    # to flag if the rrt is successful or not (if the target has been reached or not)
        self.nodes = []         # we remove the previous graph

        # TODO

    def compute_path(self):
        """ function that compute a path in the tree to go from the starting node
            to the goal node
        """
        # TODO
        pass

    def improve_path(self, my_map:SegMap):
        """ function to improve the computed path by removing nodes that are not necessary
            (free trajectory)
            PARAMETERS:
                my_map: (environment.seg_map.SegMap) the known map
        """
        # TODO
        pass

    def clear(self):
        """ function to remove the previous path and graph
        """
        self.path.clear()
        self.nodes.clear()

    def __str__(self) -> str:
        """ function convert a RTTStar into a string
        """
        ret_str = ""
        for i, n in enumerate(self.nodes):
            ret_str += f"{i} : {n}\n"
        return ret_str
