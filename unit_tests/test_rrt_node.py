import unittest

from tp_rrt.rrt_star import Node


class TestRRTNode(unittest.TestCase):

    def setUp(self):
        pass

    def test_step_from_to(self):
        self.assertAlmostEqual(Node(1.7, 0).x, Node.step_from_to(Node(1, 1), Node(3, 3), 1).x, 1)
        self.assertAlmostEqual(Node(0, 1.7).z, Node.step_from_to(Node(1, 1), Node(3, 3), 1).z, 1)

        self.assertAlmostEqual(Node(2.6, 0).x, Node.step_from_to(Node(3, 3), Node(1, 1), 0.5).x, 1)
        self.assertAlmostEqual(Node(0, 2.6).z, Node.step_from_to(Node(3, 3), Node(1, 1), 0.5).z, 1)

        self.assertAlmostEqual(Node(4.9, 0).x, Node.step_from_to(Node(4, 4), Node(6, 3), 1).x, 1)
        self.assertAlmostEqual(Node(0, 3.6).z, Node.step_from_to(Node(4, 4), Node(6, 3), 1).z, 1)

        self.assertAlmostEqual(Node(3.5, 0).x, Node.step_from_to(Node(4, 4), Node(1, 5), 0.5).x, 1)
        self.assertAlmostEqual(Node(0, 4.2).z, Node.step_from_to(Node(4, 4), Node(1, 5), 0.5).z, 1)

        self.assertEqual(Node(1, 5).x, Node.step_from_to(Node(4, 4), Node(1, 5), 10).x, 1)
        self.assertEqual(Node(1, 5).z, Node.step_from_to(Node(4, 4), Node(1, 5), 10).z, 1)

        print("[OK] Node::step_from_to()")


if __name__ == "__main__":
    unittest.main()
