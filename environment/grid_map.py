# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a GridMap class and a GridCell class
"""
from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D


class GridCell:
    """ class to handle a cell in the grid map
    """
    x: int      # (int) x coordinate of the cell in the map 
    z: int      # (int) z coordinate of the cell in the map
    val: float  # (float) value of the cell (obstacle or free)

    def __init__(self, x:int=0, z:int=0, val:float=0):
        """ constructor of the class
        PARAMETERS:
            x: (int) x coordinate of the cell in the map
            z: (int) z coordinate of the cell in the map
            val: (number) value of the cell (obstacle or free)
        """
        self.x = x  # x coordinate of the cell in the map
        self.z = z  # y coordinate of the cell in the map
        self.val = val  # value of the cell (obstacle or free)


class GridMap:
    """ class to handle a grid map

    METHODS:
        init_map()
        def compute_map(environment)
    """
    width: float           # (number in m)  the width of the map
    height: float          # (number in m) the height of the map
    nb_cell_x: int         # (int) number of cells according to the x-axis
    nb_cell_z: int         # (int) number of cells according to the z-axis
    cells: List[List[GridCell]]  # (list of GridCell)  list of cells
    size_x: float          # (number in m) x size of a cell
    size_z: float          # (number in m) z size of a cell

    def __init__(self):
        """ constructor of the class
        """
        self.width = None  # m  the width of the map
        self.height = None  # m  the height of the map
        self.nb_cell_x = None  # -  number of cells according to the x-axis
        self.nb_cell_z = None  # -  number of cells according to the z-axis
        self.cells = None  # -  list of cells
        self.size_x = None  # m  x size of a cell
        self.size_z = None  # m  z size of a cell

    def init_map(self):
        """ function to initialize the map
        """
        self.width = 12.0
        self.height = 12.0
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

    def __setitem__(self, key, value):
        if isinstance(key, tuple):
            li, col = key
            self.cells[li][col] = value
        else:
            self.cells[key] = value

    def __getitem__(self, key):
        if isinstance(key, tuple):
            li, col = key
            return self.cells[li][col]
        else:
            return self.cells[key]
