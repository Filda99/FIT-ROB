"""
    Project: Robotics Lecture
    File: grid_map.py
    Description: This file contains the GridMap class and its associated methods for representing the environment's grid map.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""
from typing import List  # for python 3.8 compatibility


class GridCell:
    """ 
        Class to handle a cell in the grid map.
        Attributes:
            x (int): The x coordinate of the cell in the map.
            z (int): The z coordinate of the cell in the map.
            val (float): The value of the cell (obstacle or free).
    """
    x: int      # (int) x coordinate of the cell in the map 
    z: int      # (int) z coordinate of the cell in the map
    val: float  # (float) value of the cell (obstacle or free)

    def __init__(self, x:int=0, z:int=0, val:float=0):
        """
            Constructor initializes the coordinates and value of the cell.
            Parameters:
                x (int): The x coordinate of the cell in the map.
                z (int): The z coordinate of the cell in the map.
                val (float): The value of the cell (obstacle or free).
        """
        self.x = x  # x coordinate of the cell in the map
        self.z = z  # y coordinate of the cell in the map
        self.val = val  # value of the cell (obstacle or free)


class GridMap:
    """
        Class to handle a grid map.
        Attributes:
            width (float): The width of the map in meters.
            height (float): The height of the map in meters.
            nb_cell_x (int): The number of cells along the x-axis.
            nb_cell_z (int): The number of cells along the z-axis.
            cells (List[List[GridCell]]): A list of lists of GridCell objects representing the map.
            size_x (float): The size of a cell along the x-axis in meters.
            size_z (float): The size of a cell along the z-axis in meters.
    """
    width: float           # (number in m)  the width of the map
    height: float          # (number in m) the height of the map
    nb_cell_x: int         # (int) number of cells according to the x-axis
    nb_cell_z: int         # (int) number of cells according to the z-axis
    cells: List[List[GridCell]]  # (list of GridCell)  list of cells
    size_x: float          # (number in m) x size of a cell
    size_z: float          # (number in m) z size of a cell

    def __init__(self):
        """
            Constructor initializes the attributes of the grid map.
        """
        self.width = 0  # m  the width of the map
        self.height = 0  # m  the height of the map
        self.nb_cell_x = 0  # -  number of cells according to the x-axis
        self.nb_cell_z = 0  # -  number of cells according to the z-axis
        self.cells = []  # -  list of cells
        self.size_x = 0  # m  x size of a cell
        self.size_z = 0  # m  z size of a cell

    def init_map(self):
        """
            Method sets the width and height of the map and initializes other attributes.
        """
        self.width = 80.0
        self.height = 80.0
        self.nb_cell_x = 60
        self.nb_cell_z = 60

        self.cells = []
        for z in range(0, self.nb_cell_z):
            self.cells.append([])
            for x in range(0, self.nb_cell_x):
                c = GridCell()
                c.x = x
                c.z = z
                c.val = 0.0
                self.cells[z].append(c)

        self.size_x = self.width / self.nb_cell_x
        self.size_z = self.height / self.nb_cell_z