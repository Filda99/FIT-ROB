# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
This file provides a SegMap class
"""
from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D
from geometry.segment import Segment2D


class SegMap:
    """ Map class to handle a map of the environment
        Here the map is a set of segments
    """
    width: float                   # (number in m) the width of the map
    height: float                  # (number in m) the width of the map
    obstaclesSeg: List[Segment2D]  # (list of Segment2D) the list that will contain the set of segments


    def __init__(self):
        """ constructor of the class
        """
        self.width = None
        self.height = None
        self.obstaclesSeg = None  # the list that will contain the set of segments

    def init_map(self):
        """ initialisation of the map
            definition of the width and the height
        """
        self.width = 12.0
        self.height = 12.0

    def compute_map(self, _):
        """ function to compute the map
            in this version, the map corresponds to a set of segments 2D
            The map is not computed, but hard-coded
            PARAMETERS:
                _: non-used variable, just to have the same prototype as the grid map and the cost map
        """
        self.obstaclesSeg = []
        offset = 1

        pts = [[0.5, 0.5], [9.5, 0.5], [9.5, 1.5], [6.5, 1.5], [6.5, 6.5], [8.5, 6.5],
               [8.5, 5.5], [7.5, 5.5], [7.5, 2.5], [9.5, 2.5], [9.5, 7.5], [3.5, 7.5],
               [3.5, 8.5], [9.5, 8.5], [9.5, 9.5], [0.5, 9.5], [0.5, 8.5], [2.5, 8.5],
               [2.5, 7.5], [0.5, 7.5], [0.5, 6.5], [3.5, 6.5], [3.5, 5.5], [0.5, 5.5],
               [0.5, 4.5], [3.5, 4.5], [3.5, 3.5], [0.5, 3.5], [0.5, 2.5], [5.5, 2.5],
               [5.5, 1.5], [0.5, 1.5]]

        point = 0
        for point in range(0, len(pts) - 1):
            self.obstaclesSeg.append(Segment2D(Point2D(pts[point][0] + offset, pts[point][1] + offset),
                                               Point2D(pts[point + 1][0] + offset, pts[point + 1][1] + offset)))
        self.obstaclesSeg.append(Segment2D(Point2D(pts[point + 1][0] + offset, pts[point + 1][1] + offset),
                                           Point2D(pts[0][0] + offset, pts[0][1] + offset)))
