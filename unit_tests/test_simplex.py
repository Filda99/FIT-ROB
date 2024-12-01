import copy
import unittest

from geometry.point import Point2D
from tp_icp.simplex import Simplex
from tp_icp.vertex import Vertex
from robot.pose import Pose3D
from tp_icp.association import Association


class TestSimplex(unittest.TestCase):

    def setUp(self):
        # run before every single tests

        self.model1 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]
        self.scene1 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]

        self.model2 = [Point2D(3, 3), Point2D(2, 2), Point2D(1, 1)]
        self.scene2 = [Point2D(1, 1), Point2D(2, 2), Point2D(3, 3)]

    def test_sort_vertices(self):
        simplex = Simplex()
        vertices = [Vertex(Pose3D(), 1), Vertex(Pose3D(), 2), Vertex(Pose3D(), 0), Vertex(Pose3D(), 3)]
        simplex.vertices = copy.deepcopy(vertices)

        simplex.sort_vertices()

        for i, vertex in enumerate(simplex.vertices):
            self.assertEqual(vertex.cost, i)
        print("[OK] Simplex::sort_vertices()")

    def test_update_cost(self):

        simplex = Simplex()
        vertices = [Vertex(Pose3D(1, 2, 3)), Vertex(Pose3D(3, 4, 1)), Vertex(Pose3D(5, 6, 0.5)),
                    Vertex(Pose3D(7, 8, 5.3))]
        simplex.vertices = copy.deepcopy(vertices)

        association = [Association(0, 1), Association(1, 2), Association(2, 0)]

        costs = [11.263, 14.657, 23.188, 28.605]

        simplex.update_cost(self.model2, self.scene2, association)

        for i, vertex in enumerate(simplex.vertices):
            self.assertAlmostEqual(vertex.cost, costs[i], 3)
        print("[OK] Simplex::update_cost()")

    def test_get_centroid(self):
        simplex = Simplex()
        simplex.vertices = [Vertex(Pose3D(1, 2, 3)), Vertex(Pose3D(4, 5, 6)), Vertex(Pose3D(7, 8, 9)),
                            Vertex(Pose3D(10, 11, 12))]
        centroid = simplex.get_centroid()
        self.assertEqual(centroid.pose.x, 4)
        self.assertEqual(centroid.pose.z, 5)
        self.assertEqual(centroid.pose.theta, 6)
        print("[OK] Simplex::get_centroid()")


if __name__ == "__main__":
    unittest.main()
