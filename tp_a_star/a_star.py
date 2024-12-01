# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides an AStar class (A* algorithm)
"""
from typing import List  # for python 3.8 compatibility

from tp_a_star.node import Node
from geometry.point import Point2D
from math import pow
from math import sqrt
from environment.grid_map import GridMap


class AStar:
    """ class to handle the A* algorithm
    ATTRIBUTES:
        self.nodes: (list of tp_a_star.node.Node) the nodes of the A*
        self.path: (list of geometry.point.Point2D) the path to go from the starting point to the target
        self.nb_z: (int) the number of cells in the x-axis
        self.nb_x: (int) the number of cells in the z-axis
        self.sizeX: (number in m) the size of a cell in the x-axis
        self.sizeZ: (number in m) the size of a cell in the z-axis
        self.width: (number in m) the total width of the map (environment, grid)
        self.height: (number in m) the total height of the map (environment, grid)
        self.opened: (list of tp_a_star.node.Node) the opened list to store all the opened nodes
        self.closed: (list of tp_a_star.node.Node) the closed list to store all the closed nodes
        self.start: (geometry.point.Point2D) the starting point for the shortest path (index in the grid)
        self.target: (geometry.point.Point2D) the target/goal for the shortest path (index in the grid)
    """
    nodes: List[Node]
    path: List[Point2D]
    nb_z: int
    nb_x: int
    sizeX: float
    sizeZ: float
    width: float
    height: float
    opened: List[Node]
    closed: List[Node]
    start: Point2D
    target: Point2D

    def __init__(self, grid_map:GridMap):
        """ class to handle the A* algorithm
        PARAMETERS:
            grid_map: (environment.grid_map.GridMap) the grid map we want to compute path in
        """
        self.nodes = [[Node()]]                 # List of nodes
        self.path = []                  # List of positions (Point2D)
        self.nb_z = grid_map.nb_cell_z  # number of cells in the x-axis
        self.nb_x = grid_map.nb_cell_x  # number of cells in the z-axis
        self.sizeX = grid_map.size_x    # m : size of a cell in the x-axis
        self.sizeZ = grid_map.size_z    # m : size of a cell in the z-axis
        self.width = grid_map.width     # m : total width of the map (environment, grid)
        self.height = grid_map.height   # m : total height of the map (environment, grid)
        # initialization of the grid of nodes
        #   z first then the x
        self.nodes.clear()
        for z in range(0, self.nb_z):
            self.nodes.append([])
            for x in range(0, self.nb_x):
                # The grid is a 2D array of nodes (defined in node.py)
                c = Node()                  # each cell corresponds to a node
                c.position = Point2D(x, z)  # updating the actual position of the node (position in the grid, index)
                # initialization of the costs
                c.s_cost = float('inf')
                c.h_cost = float('inf')
                c.f_cost = float('inf')
                # update the cell as walkable (not an obstacle) or not (not walkable => an obstacle in the map)
                if grid_map.cells[z][x].val == 1:
                    c.walkable = False
                else:
                    c.walkable = True
                # the initialized node is added to the grid
                self.nodes[z].append(c)
        self.opened = []         # the opened list to store all the opened nodes
        self.closed = []         # the closed list to store all the opened nodes
        self.start = Point2D()   # the starting point for the shortest path (index in the grid)
        self.target = Point2D()  # the target/goal for the shortest path (index in the grid)

    def reset(self):
        """ reset function to reset the A* (the values of the nodes)
        """
        for z in range(0, self.nb_z):
            for x in range(0, self.nb_x):
                # for all the set, the costs are reset
                self.nodes[z][x].s_cost = float('+inf')
                self.nodes[z][x].h_cost = float('+inf')
                self.nodes[z][x].f_cost = float('+inf')
                # the attributes of the nodes are reset
                self.nodes[z][x].is_opened = False
                self.nodes[z][x].is_closed = False
                self.nodes[z][x].is_path = False
        # and the lists are emptied
        self.closed.clear()
        self.opened.clear()
        self.path.clear()

    def find_path(self, start_pos:Point2D, target_pos:Point2D):
        """ The A* algorithm, find the shortest path in the grid from start to target
            PARAMETERS:
                start_pos : (geometry.point.Point2D) the starting cell (indexes of the grid)
                target_pos : (geometry.point.Point2D) the target cell (indexes of the grid)
        """
        self.reset()  # reset the A* first, just in case

        # set the starting point and the target
        self.start = start_pos
        self.target = target_pos

        # TODO

    def get_world_coordinates_from_index(self, index:Point2D) -> Point2D:
        """ This function returns a Point2D with world coordinates according to the index (grid) coordinates
            PARAMETERS:
                index: (geometry.point.Point2D ) Point2D corresponding to index in the grid
            RETURN:
                geometry.point.Point2D : the world coordinates (in m)
        """
        world = Point2D()  # creates a new Point2D
        # TODO
        return world  # returns the new Point2D

    def update_node(self, pos_parent:Point2D, dx:int, dz:int):
        """ This function updates a node
            PARAMETERS:
                pos_parent: (geometry.point.Point2D) the position of the parent node
                dx: (int) the x position of the node to update according to the parent node (-1,0,1)
                dz: (int) the z position of the node to update according to the parent node (-1,0,1)
        """
        # TODO
        pass

    def heuristic(self, position:Point2D) -> float:
        """ This function computes the heuristic from a given position (the euclidean distance*10
                between the position and the target)
            PARAMETERS:
                position: (geometry.point.Point2D) the position we want to heuristic of (indexes in the grid)
            RETURN:
                the euclidean distance*10 (number) from the position to the target
        """
        # TODO
        return 0
