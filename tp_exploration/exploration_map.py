# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Map class and a Cell class for the exploration workshop
"""

from geometry.segment import Segment2D
from geometry.point import Point2D
from environment.grid_map import GridMap
from environment.grid_map import GridCell
from math import cos
from math import sin


class ExplorationGridCell(GridCell):
    """ class to handle a cell in the exploration grid map
    ATTRIBUTES:
        x: (int) x coordinate of the cell in the map
        z: (int) z coordinate of the cell in the map
        val: (number) value of the cell (0: free, 1: obstacle, 0.5: unknown, 0.7 enlarged obstacle)
        cost: cost of a cell (distance to the closest frontier)
        is_frontier: (boolean) is the cell a frontier or not
    """
    x: int
    z: int
    val: float
    cost: float
    is_frontier: bool

    def __init__(self, x=0, z=0, val=0):
        """ constructor of the class
        PARAMETERS:
            # TODO
        """
        GridCell.__init__(self, x, z, val)   # constructor of the parent class
        # note that the value of the cell can be:
        #   0: free, 1: obstacle, 0.5: unknown, 0.7 enlarged obstacle
        self.cost = float('inf')  # cost of a cell (distance to the closest frontier)
        self.is_frontier = False  # bool  is the cell a frontier or not


class ExplorationGridMap(GridMap):
    """ class to handle an exploration map
    ATTRIBUTES:
        max_cost: (number) maximal current cost of the cell
        changed: (boolean) indicates if the cost of a cell has been updated
    """
    
    def __init__(self):
        """ constructor of the class
        """
        GridMap.__init__(self)  # constructor of the parent class
        self.max_cost = None   # maximal cost of a cell (distance to a frontier)
        self.changed = False   # indicates if the cost of a cell has been updated

    def init_map(self):
        """ function to initialize the map for the workshop
        """
        self.width = 12.0
        self.height = 12.0
        self.nb_cell_x = 60
        self.nb_cell_z = 60
        self.max_cost = 1

        self.cells = []
        for z in range(0, self.nb_cell_z):
            self.cells.append([])
            for x in range(0, self.nb_cell_x):
                c = ExplorationGridCell()
                c.x = x
                c.z = z
                c.val = 0.5              # the initialized cells are unknown
                self.cells[z].append(c)

        self.size_x = self.width / self.nb_cell_x
        self.size_z = self.height / self.nb_cell_z

    def add_measurements(self, robot_pose, measurements):
        """ function to add a measurements set in the map
            it uses the add_obstacle function
            PARAMETERS:
                robot_pose: (robot.pose.Pose3D) the robot pose (x, z, theta)
                measurements: (robot.lidar.LiDARMeasurement) the LiDAR measurements set
        """
        # TODO
        pass

    def add_obstacle(self, r_x, r_z, obs_x, obs_z):
        """ function to add an obstacle in the map
            it uses the free_path and enlarge_obstacle functions
            PARAMETERS:
                r_x: (number in m) the robot x position
                r_z: (number in m) the robot z position
                obs_x: (number in m) the obstacle x position
                obs_z: (number in m) the obstacle z position
        """
        # TODO
        pass

    def free_path(self, r_x, r_z, obs_x, obs_z):
        """ function that marks all the cells between the robot and the detected obstacle
            as a free cell (without obstacle)
            it uses the update_cell function
            PARAMETERS:
                r_x: (number in m) the robot x position
                r_z: (number in m) the robot z position
                obs_x: (number in m) the obstacle x position
                obs_z: (number in m) the obstacle z position
        """
        # TODO
        pass

    def update_cell(self, cx, cz, laser, enter):
        """ this is a recursive function
            PARAMETERS:
                cx: (int) the current cell x index (in the grid)
                cz: (int) the current cell z index (in the grid)
                laser: (geometry.segment.Segment2D) Segment2D that corresponds to the laser ray
                    note that the coordinates of the segment are in m (coordinates in the world frame)
                enter : where we came from (recursive function). The value of enter can be:
                    # 0: first cell
                    # 1: enter from north
                    # 2: enter from south
                    # 3: enter from east
                    # 4: enter from west
        """
        # TODO
        pass

    def remove_isolated_frontiers(self):
        """ remove the isolated frontiers
            a frontier must have at least one free neighbour (a cell with a value 0)
        """
        # TODO
        pass

    def compute_costs(self):
        """ function to compute the cost (distance between the cell and the closest frontier)
            of all the cells
            it uses init_costs, and the 4 compute_X_Y functions

            if the cell is a frontier, the cost is 0
        """
        self.init_costs()
        # TODO

    def init_costs(self):
        """ function that initialises the costs of all the cells with 'inf'
        """
        for line in self.cells:
            for cell in line:
                cell.cost = float('inf')
        self.max_cost = 0
        self.changed = True  # To indicates that the cell cost have changed

    def compute_nw_se(self):
        """ function to compute the cost cells from the top left corner to the bottom right corner
        """
        # TODO
        pass

    def compute_ne_sw(self):
        """ function to compute the cost cells from the top right corner to the bottom left corner
        """
        # TODO
        pass

    def compute_sw_ne(self):
        """ function to compute the cost cells from the bottom left corner to the top right corner
        """
        # TODO
        pass

    def compute_se_nw(self):
        """ function to compute the cost cells from the bottom right corner to the top left corner
        """
        # TODO
        pass

    def enlarge_obstacle(self, i_obs_x, i_obs_z):
        """ function to enlarge the obstacles
            a cell which has an obstacle as neighbour must have a 0.7 value (avoid the robot to
            move too close to the obstacles)
            PARAMETERS:
                i_obs_x: (int) the x coordinate of the obstacle (index in the grid)
                i_obs_z: (int) the z coordinate of the obstacle (index in the grid)
        """
        # TODO
        pass
