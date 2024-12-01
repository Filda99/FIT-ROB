import unittest

from geometry.point import Point2D
from tp_a_star.a_star import AStar
from environment.grid_map import GridMap
from environment.grid_map import GridCell
from tp_a_star.node import Node


class TestAStar(unittest.TestCase):

    def setUp(self):
        self.grid_map = GridMap()

        self.grid_map.width = 5
        self.grid_map.height = 7

        self.grid_map.nb_cell_x = 15
        self.grid_map.nb_cell_z = 21

        self.grid_map.cells = []
        for z in range(0, self.grid_map.nb_cell_z):
            self.grid_map.cells.append([])
            for x in range(0, self.grid_map.nb_cell_x):
                c = GridCell()
                c.x = x
                c.z = z
                c.val = 0.0
                self.grid_map.cells[z].append(c)

        self.grid_map.size_x = self.grid_map.width / self.grid_map.nb_cell_x
        self.grid_map.size_z = self.grid_map.height / self.grid_map.nb_cell_z

        self.a_star = AStar(self.grid_map)
        pass

    def test_get_world_coordinates_from_index(self):
        self.a_star.sizeX = 10
        self.a_star.sizeZ = 20

        test1 = self.a_star.get_world_coordinates_from_index(Point2D(5, 10))

        self.assertAlmostEqual(test1.x, 55, 1)
        self.assertAlmostEqual(test1.z, 210, 1)

        test2 = self.a_star.get_world_coordinates_from_index(Point2D(10, 5))

        self.assertAlmostEqual(test2.x, 105, 1)
        self.assertAlmostEqual(test2.z, 110, 1)

        print("[OK] AStar::get_world_coordinates_from_index()")

    def test_heuristic(self):
        self.a_star.target = Point2D(5, 9)

        test1 = self.a_star.heuristic(Point2D(1, 7))
        self.assertAlmostEqual(test1, 44.7, 1)

        test2 = self.a_star.heuristic(Point2D(10, 3))
        self.assertAlmostEqual(test2, 78.1, 1)

        print("[OK] AStar::heuristic()")

    def test_update_node(self):

        self.a_star.nodes = []
        for z in range(3):
            self.a_star.nodes.append([])
            for x in range(2):
                self.a_star.nodes[z].append(Node())
                self.a_star.nodes[z][x].position = Point2D(x, z)
                self.a_star.nodes[z][x].walkable = True
                self.a_star.nodes[z][x].h_cost = z*x
                self.a_star.nodes[z][x].s_cost = z+x
                self.a_star.nodes[z][x].f_cost = z*x + z+x

        self.a_star.nodes[0][0].walkable = False

        self.a_star.nodes[1][0].h_cost = float('inf')
        self.a_star.nodes[1][0].s_cost = float('inf')
        self.a_star.nodes[1][0].f_cost = float('inf')
        self.a_star.nodes[1][0].parent_position = Point2D(0, 2)
        self.a_star.update_node(Point2D(0, 0), 0, 1)
        self.assertEqual(self.a_star.nodes[1][0].h_cost, 10)
        self.assertEqual(self.a_star.nodes[1][0].s_cost, 10)
        self.assertEqual(self.a_star.nodes[1][0].f_cost, 20)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.x, 0)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.z, 0)

        self.a_star.nodes[1][0].h_cost = float('inf')
        self.a_star.nodes[1][0].s_cost = float('inf')
        self.a_star.nodes[1][0].f_cost = float('inf')
        self.a_star.nodes[1][0].parent_position = Point2D(0, 2)
        self.a_star.update_node(Point2D(1, 0), -1, 1)
        self.assertEqual(self.a_star.nodes[1][0].h_cost, 10)
        self.assertEqual(self.a_star.nodes[1][0].s_cost, 15)
        self.assertEqual(self.a_star.nodes[1][0].f_cost, 25)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.x, 1)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.z, 0)

        self.a_star.nodes[1][0].h_cost = float('inf')
        self.a_star.nodes[1][0].s_cost = float('inf')
        self.a_star.nodes[1][0].f_cost = float('inf')
        self.a_star.nodes[1][0].parent_position = Point2D(0, 2)
        self.a_star.update_node(Point2D(1, 1), -1, 0)
        self.assertEqual(self.a_star.nodes[1][0].h_cost, 10)
        self.assertEqual(self.a_star.nodes[1][0].s_cost, 12)
        self.assertEqual(self.a_star.nodes[1][0].f_cost, 22)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.x, 1)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.z, 1)

        self.a_star.nodes[1][0].h_cost = 100
        self.a_star.nodes[1][0].s_cost = 110
        self.a_star.nodes[1][0].f_cost = 120
        self.a_star.nodes[1][0].parent_position = Point2D(1, 1)
        self.a_star.update_node(Point2D(0, 0), -1, 0)
        self.assertEqual(self.a_star.nodes[1][0].h_cost, 100)
        self.assertEqual(self.a_star.nodes[1][0].s_cost, 110)
        self.assertEqual(self.a_star.nodes[1][0].f_cost, 120)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.x, 1)
        self.assertEqual(self.a_star.nodes[1][0].parent_position.z, 1)

        print("[OK] AStar::update_node()")

    def test_find_path(self):

        self.a_star.nb_z = 6
        self.a_star.nb_x = 11
        self.a_star.sizeX = 2
        self.a_star.sizeZ = 2
        self.a_star.width = 12
        self.a_star.height = 22

        self.a_star.nodes = []
        for z in range(6):
            self.a_star.nodes.append([])
            for x in range(11):
                self.a_star.nodes[z].append(Node())
                self.a_star.nodes[z][x].position = Point2D(x, z)
                self.a_star.nodes[z][x].walkable = True

        self.a_star.nodes[2][4].walkable = False
        self.a_star.nodes[2][5].walkable = False
        self.a_star.nodes[2][6].walkable = False

        self.a_star.find_path(Point2D(7, 4), Point2D(4, 1))

        self.assertEqual(len(self.a_star.path), 6)

        self.assertTrue(4*2 <= self.a_star.path[0].x <= 5*2)
        self.assertTrue(1*2 <= self.a_star.path[0].z <= 2*2)

        self.assertTrue(5*2 <= self.a_star.path[1].x <= 6*2)
        self.assertTrue(1*2 <= self.a_star.path[1].z <= 2*2)

        self.assertTrue(6*2 <= self.a_star.path[2].x <= 7*2)
        self.assertTrue(1*2 <= self.a_star.path[2].z <= 2*2)

        self.assertTrue(7*2 <= self.a_star.path[3].x <= 8*2)
        self.assertTrue(2*2 <= self.a_star.path[3].z <= 3*2)

        self.assertTrue(7*2 <= self.a_star.path[4].x <= 8*2)
        self.assertTrue(3*2 <= self.a_star.path[4].z <= 4*2)

        self.assertTrue(7*2 <= self.a_star.path[5].x <= 8*2)
        self.assertTrue(4*2 <= self.a_star.path[5].z <= 5*2)

        print("[OK] AStar::find_path()")


if __name__ == "__main__":
    unittest.main()
