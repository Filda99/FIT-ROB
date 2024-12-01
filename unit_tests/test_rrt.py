import unittest

from tp_rrt.rrt_star import RRTStar
from tp_rrt.rrt_star import Node
from environment.seg_map import SegMap
from geometry.segment import Segment2D
from geometry.point import Point2D


class TestRRTStar(unittest.TestCase):

    def setUp(self):
        # run before every single tests
        self.map = SegMap()

    def test_get_rand_node(self):

        self.map.width = 3.0
        self.map.height = 2.0

        old_x = -1
        old_z = -1
        for _ in range(100):
            node = RRTStar.get_rand_node(self.map)
            self.assertTrue(0.0 <= node.x <= 3.0)
            self.assertTrue(0.0 <= node.z <= 2.0)
            self.assertNotEqual(old_x, node.x)
            self.assertNotEqual(old_z, node.z)
            old_x = node.x
            old_z = node.z

        print("[OK] RRTStar::get_rand_node()")

    def test_trajectory_free(self):
        self.map.width = 3.0
        self.map.height = 2.0
        self.map.obstaclesSeg = [Segment2D(Point2D(2, 2), Point2D(5, 4)),
                                 Segment2D(Point2D(1, 5), Point2D(3, 4))]

        self.assertTrue(RRTStar.trajectory_free(self.map, Point2D(1, 1), Point2D(5, 1)))
        self.assertFalse(RRTStar.trajectory_free(self.map, Point2D(1, 3), Point2D(4, 2)))
        self.assertTrue(RRTStar.trajectory_free(self.map, Point2D(3, 3), Point2D(5, 5)))
        self.assertFalse(RRTStar.trajectory_free(self.map, Point2D(1, 5), Point2D(3, 4)))

        print("[OK] RRTStar::trajectory_free()")

    def test_build_rrt(self):
        self.map.width = 7.0
        self.map.height = 7.0
        self.map.obstaclesSeg = [Segment2D(Point2D(2, 2), Point2D(5, 4)),
                                 Segment2D(Point2D(2, 2), Point2D(2, 5)),
                                 Segment2D(Point2D(2, 5), Point2D(5, 4))]

        rrt = RRTStar()

        for _ in range(1000):
            rrt.epsilon = 1
            rrt.max_nodes = 100
            rrt.rand_nodes_before_target = 20
            rrt.build_rrt(self.map, Point2D(1, 4), Point2D(5, 2))
            self.assertLessEqual(len(rrt.nodes), rrt.max_nodes)

            for node in rrt.nodes[1:]:
                self.assertTrue(RRTStar.trajectory_free(self.map, node, rrt.nodes[node.parent]))
                self.assertLessEqual(Point2D.distance(node, rrt.nodes[node.parent]), rrt.epsilon+0.001)

            self.assertEqual(rrt.nodes[-1].x, 5)
            self.assertEqual(rrt.nodes[-1].z, 2)

        rrt.epsilon = 0.1
        rrt.max_nodes = 200
        rrt.rand_nodes_before_target = 20
        rrt.build_rrt(self.map, Point2D(3, 4), Point2D(5, 2))
        self.assertEqual(len(rrt.nodes), rrt.max_nodes)

        print("[OK] RRTStar::build_rrt()")

    def test_chose_parent(self):
        self.map.width = 7.0
        self.map.height = 7.0
        self.map.obstaclesSeg = [Segment2D(Point2D(2, 2), Point2D(4, 5))]

        rrt = RRTStar()
        rrt.radius = 1.5

        n0 = Node(2, 3)
        n1 = Node(1, 5)
        n2 = Node(3, 4)
        n3 = Node(1, 1)

        n0.id = 0
        n0.cost = 0

        n1.id = 1
        n1.parent = n0.id
        n1.cost = n0.cost + Point2D.distance(n1, n0)

        n2.id = 2
        n2.parent = n1.id
        n2.cost = n1.cost + Point2D.distance(n2, n1)

        n3.id = 3
        n3.parent = n0.id
        n3.cost = n0.cost + Point2D.distance(n3, n0)
        rrt.nodes = [n0, n1, n2, n3]

        n_test = Node(3, 2)
        updated_node = rrt.choose_parent(self.map, rrt.nodes[3], n_test)
        self.assertAlmostEqual(4.5, updated_node.cost, 1)
        self.assertEqual(3, updated_node.parent)

        n_test = Node(1, 3)
        updated_node = rrt.choose_parent(self.map, rrt.nodes[3], n_test)
        self.assertAlmostEqual(1, updated_node.cost, 1)
        self.assertEqual(0, updated_node.parent)

        n_test = Node(2, 5)
        updated_node = rrt.choose_parent(self.map, rrt.nodes[3], n_test)
        self.assertAlmostEqual(3.2, updated_node.cost, 1)
        self.assertEqual(1, updated_node.parent)

        print("[OK] RRTStar::chose_parent()")

    def test_compute_path(self):

        self.map.width = 7.0
        self.map.height = 7.0

        rrt = RRTStar()

        nodes = [Node(5, 2),
                 Node(3, 1),
                 Node(7, 3),
                 Node(4, 3),
                 Node(3, 4),
                 Node(5, 5),
                 Node(2, 3),
                 Node(7, 5),
                 Node(1, 5),
                 Node(3, 7)]
        for i, n in enumerate(nodes):
            n.id = i

        nodes[0].parent = None
        nodes[1].parent = nodes[0].id
        nodes[2].parent = nodes[5].id
        nodes[3].parent = nodes[0].id
        nodes[4].parent = nodes[3].id
        nodes[5].parent = nodes[4].id
        nodes[6].parent = nodes[3].id
        nodes[7].parent = nodes[2].id
        nodes[8].parent = nodes[6].id
        nodes[9].parent = nodes[5].id

        rrt.nodes = []
        for node in nodes:
            rrt.nodes.append(node)

        rrt.succeed = True
        rrt.compute_path()

        path = [nodes[9], nodes[5], nodes[4], nodes[3], nodes[0]]

        if len(rrt.path) == len(path)-1:
            path = path[:-1]
            # pas de start
            if path[0] != rrt.path[0]:
                path.reverse()

            for i, point in enumerate(path):
                self.assertEqual(point.x, rrt.path[i].x)
                self.assertEqual(point.z, rrt.path[i].z)

        else:
            if path[0] != rrt.path[0]:
                path.reverse()

            for i, point in enumerate(path):
                self.assertEqual(point.x, rrt.path[i].x)
                self.assertEqual(point.z, rrt.path[i].z)

            self.assertEqual(len(path), len(rrt.path))

        print("[OK] RRTStar::compute_path()")

    def test_improve_path(self):
        self.map.width = 7.0
        self.map.height = 7.0
        self.map.obstaclesSeg = [Segment2D(Point2D(1, 6), Point2D(4, 5))]

        rrt = RRTStar()

        nodes = [Node(5, 2),
                 Node(3, 1),
                 Node(7, 3),
                 Node(4, 3),
                 Node(3, 4),
                 Node(5, 5),
                 Node(2, 3),
                 Node(7, 5),
                 Node(1, 5),
                 Node(3, 7)]
        for i, n in enumerate(nodes):
            n.id = i

        nodes[0].parent = None
        nodes[1].parent = nodes[0].id
        nodes[2].parent = nodes[5].id
        nodes[3].parent = nodes[0].id
        nodes[4].parent = nodes[3].id
        nodes[5].parent = nodes[4].id
        nodes[6].parent = nodes[3].id
        nodes[7].parent = nodes[2].id
        nodes[8].parent = nodes[6].id
        nodes[9].parent = nodes[5].id

        rrt.nodes = []
        for node in nodes:
            rrt.nodes.append(node)

        rrt.succeed = True
        rrt.path = [nodes[9], nodes[5], nodes[4], nodes[3], nodes[0]]
        rrt.improve_path(self.map)

        self.assertEqual(len(rrt.path), 3)
        for i, point in enumerate(rrt.path[:-1]):
            self.assertTrue(RRTStar.trajectory_free(self.map, point, rrt.path[i+1]))

        print("[OK] RRTStar::improve_path()")


if __name__ == "__main__":
    unittest.main()
