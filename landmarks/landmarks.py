"""
    Project: ROBa project
    File: landmarks.py
    Description: This file contains functions and classes related to landmarks in the robotics project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""

from geometry.point import Point2D

_landmarks: list[Point2D] = [Point2D(10, 10), Point2D(50, 70), Point2D(25, 30), Point2D(10, 65), Point2D(5, 40), Point2D(50, 10), Point2D(70, 50), Point2D(70, 12)]

def get_landmarks():
    return _landmarks