import unittest

from geometry.point import Point2D
from tp_icp.simplex import Vertex
from tp_icp.association import Association
from robot.pose import Pose3D


class TestVertex(unittest.TestCase):

    def setUp(self):
        # run before every single tests
        self.model1 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]
        self.scene1 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]

        self.model2 = [Point2D(3, 3), Point2D(2, 2), Point2D(1, 1)]
        self.scene2 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]

        self.vertex = Vertex()

    def test_update_cost(self):
        association = [Association(0, 0), Association(1, 1), Association(2, 2)]
        self.vertex.update_cost(self.model1, self.scene1, association)
        self.assertAlmostEqual(0.0, self.vertex.cost, 3)

        association = [Association(0, 2), Association(1, 0), Association(2, 1)]
        self.vertex.update_cost(self.model1, self.scene1, association)
        self.assertAlmostEqual(5.6569, self.vertex.cost, 3)

        association = [Association(0, 0), Association(1, 1), Association(2, 2)]
        self.vertex.update_cost(self.model2, self.scene2, association)
        self.assertAlmostEqual(5.6568, self.vertex.cost, 3)

        self.vertex = Vertex(Pose3D(2, 3))
        self.vertex.update_cost(self.model2, self.scene2, association)
        self.assertAlmostEqual(11.0087, self.vertex.cost, 3)
        print("[OK] Vertex::update_cost()")


if __name__ == "__main__":
    unittest.main()
