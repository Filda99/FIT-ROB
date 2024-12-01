# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file defines a CostCell class and a CostMap class
"""

from math import cos
from math import sin
from typing import List  # for python 3.8 compatibility

from environment.grid_map import GridMap
from environment.grid_map import GridCell
from robot.pose import Pose3D
from robot.lidar import LiDARMeasurement


class CostCell(GridCell):
    """ class to handle the cells of a cost map
    ATTRIBUTES:
        cost : (number) the cost of the cell
    """
    cost: float

    def __init__(self, x=0, z=0, cost=0):
        """ constructor of the class
        :param x: (int) x coordinate of the cell in the map
        :param z: (int) z coordinate of the cell in the map
        :param cost: (number) the cost of the cell
        """
        GridCell.__init__(self, x, z)
        self.cost = cost  # cost of the cell (distance to the closest obstacle)

    def __str__(self) -> str:
        """ to display a CostCell as a string"""
        return f"({self.x}, {self.z}, {self.cost})"


class CostMap(GridMap):
    """ class to handle a cost map
    ATTRIBUTES:
        max_cost: (number) maximal current cost of the cells
    """
    max_cost: float

    def __init__(self):
        """ constructor of the class
        """
        GridMap.__init__(self)
        self.max_cost = None  # -  maximal current cost of the cells

    def init_cost_map(self, grid_map:GridMap):
        """ function to initialize the cost map according to a grid map
            :param grid_map: (environment.grid_map GridMap) a GridMap to compute the cost of
        """
        self.width = grid_map.width
        self.height = grid_map.height
        self.nb_cell_x = grid_map.nb_cell_x
        self.nb_cell_z = grid_map.nb_cell_z
        self.max_cost = 0

        self.cells = [[CostCell()]]
        self.cells.clear()
        for z in range(0, self.nb_cell_z):
            self.cells.append([])
            for x in range(0, self.nb_cell_x):
                c = CostCell()
                c.x = x
                c.z = z
                # if the cell corresponds to an obstacle, its cost is 0
                # otherwise the cost is initialized by +infinity
                if grid_map.cells[z][x].val == 1:
                    c.cost = 0
                else:
                    c.cost = float('inf')
                self.cells[z].append(c)

        self.size_x = self.width / self.nb_cell_x
        self.size_z = self.height / self.nb_cell_z

    def compute_cost_map(self, grid_map:GridMap):
        """ function to compute the cost map from a grid map
        :param grid_map: (environment.grid_map GridMap)
        """
        self.init_cost_map(grid_map)
        # the computation is done by going through all the cells 4 times
        # each line from the left to the right
        # each line from the right to the left
        # each row from the up to the down
        # each row from the down to the up
        # do not forget to UPDATE self.max_cost, initialized at 0
        # TODO

    def compute_west_2_east(self):
        """ function to compute the cells from the west corner to the east side
        """
        # TODO
        pass

    def compute_east_2_west(self):
        """ function to compute the cells from the east to the west side
        """
        # TODO
        pass

    def compute_south_2_north(self):
        """ function to compute the cells the top to the bottom
        """
        # TODO
        pass

    def compute_north_2_south(self):
        """ function to compute the cells from the bottom to the top
        """
        # TODO
        pass

    def cell_is_empty(self, dx:float, dz:float) -> bool:
        """ function to test if the cell corresponding to the dx and dz distances is empty or not
            dx and dz are coordinates in the environment
            PARAMETERS:
                dx: (number in m) x coordinate in the world frame
                dz: (number in m) z coordinate in the world frame
            RETURNS:
                True of False
        """
        # TODO
        return True

    def evaluate_cost(self, pose:Pose3D, measurements:List[LiDARMeasurement]) -> float:
        """ function to evaluate the cost of measurements set according to the 
            position x,z and the orientation theta
            parameters:
                pose: (robot.pose.Pose3D) the pose (x, z, theta)
                measurements: (list of robot.lidar.LiDARMeasurement) the measurements
            return:
                cost: (float)
        """
        cost = 0
        # TODO
        return cost
