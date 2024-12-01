# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides an SegEnv class, an environment based on a list of segments
"""

from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D
from geometry.segment import Segment2D


class SegEnv:
    """ SegEnv class to handle an environment composed of a set of segments
    ATTRIBUTES:
        width: (number in m) the width of the environment
        height: (number in m) the height of the environment
        segments: (list of geometry.segment Segment2D) the segments of the environment
    METHODS:
        add(segment2d)
        init_environment()
        init_environment()

    """

    width: float
    height: float
    segments: List[Segment2D]

    def __init__(self, width:float=0, height:float=0):
        """ the constructor of the class
            in this version the environment is a set of segments
            the list of segments is initialized as empty
            PARAMETERS:
                width: (number in m) the width of the environment
                height: (number in m) the height of the environment
        """
        self.width = width
        self.height = height
        self.segments = []

    def add(self, segment2d:Segment2D):
        """ a function to add a segment to the environment
        PARAMETERS:
            segment2d: (geometry.segment Segment2D) the segment you want to add
        """
        offset = 1
        segment2d.p1.x += offset
        segment2d.p2.x += offset
        segment2d.p1.z += offset
        segment2d.p2.z += offset
        self.segments.append(segment2d)

    def init_environment(self):
        """ a function to initialize the environment
        """
        self.add(Segment2D(Point2D(0, 0), Point2D(10, 0)))
        self.add(Segment2D(Point2D(10, 0), Point2D(10, 10)))
        self.add(Segment2D(Point2D(10, 10), Point2D(0, 10)))
        self.add(Segment2D(Point2D(0, 10), Point2D(0, 0)))

        self.add(Segment2D(Point2D(0, 2), Point2D(5, 2)))
        self.add(Segment2D(Point2D(0, 4), Point2D(3, 4)))
        self.add(Segment2D(Point2D(0, 6), Point2D(3, 6)))
        self.add(Segment2D(Point2D(0, 8), Point2D(2, 8)))
        self.add(Segment2D(Point2D(4, 8), Point2D(10, 8)))
        self.add(Segment2D(Point2D(7, 2), Point2D(10, 2)))
        self.add(Segment2D(Point2D(7, 2), Point2D(7, 6)))
        self.add(Segment2D(Point2D(7, 6), Point2D(8, 6)))

        self.width = 12.0
        self.height = 12.0

    def init_environment_icp(self):
        """ a function to initialize the environment
        """
        self.add(Segment2D(Point2D(2, 6), Point2D(2, 2)))
        self.add(Segment2D(Point2D(2, 2), Point2D(6, 2)))
        self.add(Segment2D(Point2D(6, 2), Point2D(6, 6)))

        self.width = 10.0
        self.height = 10.0
