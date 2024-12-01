import copy
import unittest

from environment.seg_environment import SegEnv
from environment.grid_map import GridMap
from tp_mcl.cost_map import CostMap
from robot.lidar import LiDARMeasurement
from robot.pose import Pose3D


class TestCostMap(unittest.TestCase):

    def setUp(self):
        # run before every single tests

        environment = SegEnv()  # the simulated environment
        environment.init_environment()
        self.grid_map = GridMap()  # the known grid map of the environment
        self.grid_map.init_map()
        self.grid_map.compute_map(environment)

        self.cost_map = CostMap()  # the cost map according to the grid map
        self.cost_map.init_cost_map(self.grid_map)

    def test_compute_cost_map(self):
        cost_map = copy.copy(self.cost_map)
        cost_map.compute_cost_map(self.grid_map)
        self.assertEqual(cost_map.cells[10][10].cost, 4)
        self.assertEqual(cost_map.cells[20][20].cost, 4)
        self.assertEqual(cost_map.cells[30][30].cost, 9)
        self.assertEqual(cost_map.cells[40][40].cost, 4)
        self.assertEqual(cost_map.cells[50][50].cost, 4)
        self.assertEqual(cost_map.cells[0][25].cost, 4)
        self.assertEqual(cost_map.cells[25][10].cost, 0)
        self.assertEqual(cost_map.cells[20][30].cost, 5)
        self.assertEqual(cost_map.cells[45][30].cost, 0)
        self.assertEqual(cost_map.cells[5][40].cost, 0)
        self.assertEqual(cost_map.cells[10][50].cost, 4)

        self.assertEqual(cost_map.cells[0][0].cost, 8)
        self.assertEqual(cost_map.cells[0][59].cost, 8)
        self.assertEqual(cost_map.cells[59][59].cost, 8)
        self.assertEqual(cost_map.cells[59][0].cost, 8)

        print("[OK] compute cost map")

    def test_cell_is_empty(self):
        self.assertEqual(self.cost_map.cell_is_empty(0, 0), True)
        self.assertEqual(self.cost_map.cell_is_empty(1, 1), False)
        self.assertEqual(self.cost_map.cell_is_empty(5, 9), False)
        self.assertEqual(self.cost_map.cell_is_empty(9, 5), True)

        print("[OK] cell is empty")

    def test_evaluate_cost(self):
        cost_map = copy.copy(self.cost_map)
        cost_map.max_cost = 20

        measurements = [LiDARMeasurement(3, 0),
                        LiDARMeasurement(4, 0.5),
                        LiDARMeasurement(3, 1),
                        LiDARMeasurement(2, 1.5),
                        LiDARMeasurement(1, 2),
                        LiDARMeasurement(5, 2.5),
                        LiDARMeasurement(1, 3)]

        for x_cell in cost_map.cells:
            for cell in x_cell:
                if cell.cost == float("inf"):
                    cell.cost = 2

        self.assertEqual(cost_map.evaluate_cost(Pose3D(0, 0, 0), measurements), 68)
        self.assertEqual(cost_map.evaluate_cost(Pose3D(1, 1, 1.5), measurements), 86)
        self.assertEqual(cost_map.evaluate_cost(Pose3D(5, 5, 3), measurements), 12)
        self.assertEqual(cost_map.evaluate_cost(Pose3D(5, 5, 0.7), measurements), 14)
        print("[OK] evaluate cost")


if __name__ == "__main__":
    unittest.main()
