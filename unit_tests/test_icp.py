import unittest

from tp_icp.icp import ICP
from tp_icp.association import Association
from geometry.point import Point2D
from tp_icp.vertex import Vertex
from tp_icp.simplex import Simplex
from robot.pose import Pose3D


class TestICP(unittest.TestCase):

    def test_associate_by_index(self):
        model = [Point2D(0, 0),   Point2D(0, 1),     Point2D(0, 2),   Point2D(1, 2),   Point2D(2, 2)]
        scene = [Point2D(0, 0.1), Point2D(0.1, 0.3), Point2D(2.2, 5), Point2D(2.3, 0), Point2D(1.4, 1.6)]

        associations = ICP.associate_by_index(model, scene)

        distance_solution = [0.1, 0.707, 3.720, 2.385, 0.721]

        self.assertEqual(len(associations), len(model))

        for i, ass in enumerate(associations):
            self.assertIsInstance(ass, Association)
            self.assertEqual(ass.id_scene, i)
            self.assertEqual(ass.id_model, i)
            self.assertAlmostEqual(ass.distance, distance_solution[i], 3)
        print("[OK] ICP::associate_by_index()")

    def test_associate_by_closest(self):
        model = [Point2D(0, 0),   Point2D(0, 1),     Point2D(0, 2),   Point2D(1, 2),   Point2D(2, 2)]
        scene = [Point2D(0, 0.1), Point2D(0.1, 0.3), Point2D(2.2, 5), Point2D(2.3, 0), Point2D(1.4, 1.6)]

        associations = ICP.associate_by_closest(model, scene)

        self.assertEqual(len(associations), len(model))

        id_model_solution = [0, 0, 4, 4, 3]
        id_scene_solution = [0, 1, 2, 3, 4]
        distance_solution = [0.1, 0.316, 3.007, 2.022, 0.566]

        for i, ass in enumerate(associations):
            self.assertIsInstance(ass, Association)
            self.assertEqual(ass.id_scene, id_scene_solution[i])
            self.assertEqual(ass.id_model, id_model_solution[i])
            self.assertAlmostEqual(ass.distance, distance_solution[i], 3)
        print("[OK] ICP::associate_by_closest()")

    def test_nelder_and_mead(self):
        model = [Point2D(0, 0),   Point2D(0, 1),     Point2D(0, 2),   Point2D(1, 2),   Point2D(2, 2)]
        scene = [Point2D(0, 0.1), Point2D(0.1, 0.3), Point2D(2.2, 5), Point2D(2.3, 0), Point2D(1.4, 1.6)]
        initial_simplex = Simplex()
        initial_simplex[0] = Vertex(Pose3D(0, 0, 0))
        initial_simplex[1] = Vertex(Pose3D(1, 0, 0))
        initial_simplex[2] = Vertex(Pose3D(0, 1, 0))
        initial_simplex[3] = Vertex(Pose3D(0, 0, 1))
        association = [Association(0, 1), Association(1, 2), Association(2, 3), Association(3, 4), Association(4, 0)]
        pose = ICP.nelder_and_mead(initial_simplex, model, scene, association)
        self.assertAlmostEqual(pose.x, 2.0331, 3)
        self.assertAlmostEqual(pose.z, 2.0944, 3)
        self.assertAlmostEqual(pose.theta, 2.8043, 3)
        print("[OK] ICP::nelder_and_mead()")
