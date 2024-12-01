import unittest

from tp_a_star.a_star import Node
from geometry.point import Point2D



class TestAStarNode(unittest.TestCase):

    def setUp(self):
        pass

    def test_lower_than(self):
        n0 = Node(s_cost=2, h_cost=3)
        n1 = Node(s_cost=3, h_cost=2)
        n2 = Node(s_cost=1, h_cost=2)
        n3 = Node(s_cost=4, h_cost=5)

        self.assertTrue(n0 < n1)
        self.assertTrue(n0 > n2)
        self.assertTrue(n0 < n3)
        self.assertTrue(n1 > n2)
        self.assertTrue(n1 < n3)
        self.assertTrue(n2 < n3)
        self.assertFalse(n2 < n2)

        print("[OK] Node::lower_than()")

    def test_update_costs(self):
        n0 = Node(s_cost=2, h_cost=3)
        n0.parent_position = Point2D(4, 6)

        new_parent1 = Point2D(7, 8)
        n0.update_costs(1, 7, new_parent1)
        self.assertEqual(n0.parent_position, Point2D(4, 6))
        self.assertEqual(n0.s_cost, 2)
        self.assertEqual(n0.h_cost, 3)
        self.assertEqual(n0.f_cost, 5)

        new_parent2 = Point2D(9, 10)
        n0.update_costs(1, 2, new_parent2)
        self.assertEqual(n0.parent_position, new_parent2)
        self.assertEqual(n0.s_cost, 1)
        self.assertEqual(n0.h_cost, 2)
        self.assertEqual(n0.f_cost, 3)

        new_parent3 = Point2D(10, 11)
        n0.update_costs(5, 8, new_parent3)
        self.assertEqual(n0.parent_position, new_parent2)
        self.assertEqual(n0.s_cost, 1)
        self.assertEqual(n0.h_cost, 2)
        self.assertEqual(n0.f_cost, 3)

        print("[OK] Node::update_costs()")


if __name__ == "__main__":
    unittest.main()
