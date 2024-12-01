import copy as copy
import unittest as unittest

from tp_mcl.cost_map import CostMap, CostCell
from robot.lidar import LiDARMeasurement
from robot.pose import Pose3D
from tp_mcl.monte_carlo import MonteCarloLocalization, Particle


class TestMonteCarloLocalization(unittest.TestCase):

    def setUp(self):
        # run before every single tests
        pass

    @staticmethod
    def get_grid_cost_map(width, height, nb_cell_x, nb_cell_z):
        cost_map = CostMap()

        cost_map.width = width
        cost_map.height = height
        cost_map.nb_cell_x = nb_cell_x
        cost_map.nb_cell_z = nb_cell_z

        cost_map.cells = [[CostCell()]]
        cost_map.cells.clear()
        for z in range(0, cost_map.nb_cell_z):
            cost_map.cells.append([])
            for x in range(0, cost_map.nb_cell_x):
                c = CostCell()
                c.x = x
                c.z = z
                cost_map.cells[z].append(c)
        cost_map.size_x = cost_map.width / cost_map.nb_cell_x
        cost_map.size_z = cost_map.height / cost_map.nb_cell_z

        for i in range(len(cost_map.cells)):
            for j in range(len(cost_map.cells[i])):
                if (i*j) % 2 == 0:
                    cost_map.cells[i][j].cost = 0
                else:
                    cost_map.cells[i][j].cost = 5
        cost_map.max_cost = 5
        return cost_map

    def test_init_particles(self):
        """
        Function to test the init_particles method
        """
        nb_particles = 3
        cost_map = TestMonteCarloLocalization.get_grid_cost_map(20, 30, 20, 30)
        mcl = MonteCarloLocalization()

        mcl.init_particles(cost_map, nb_particles)

        self.assertEqual(len(mcl.particles), nb_particles)

        for p in mcl.particles:
            x_p = int(p.pose.x / cost_map.size_x)
            z_p = int(p.pose.z / cost_map.size_z)
            self.assertNotEqual(cost_map.cells[z_p][x_p].cost, 0)
            self.assertTrue((1/nb_particles-0.01 <= p.weight <= 1/nb_particles+0.01) or p.weight == 0.0)
            # self.assertAlmostEqual(p.weight, 1/nb_particles, 3)
            self.assertGreaterEqual(p.pose.x, 0)
            self.assertGreaterEqual(p.pose.z, 0)
            self.assertLessEqual(p.pose.x, cost_map.width)
            self.assertLessEqual(p.pose.z, cost_map.height)

        print("[OK] Init particles")

    def test_evaluate_particles(self):
        measurements = [LiDARMeasurement(3, 0),
                        LiDARMeasurement(4, 0.5),
                        LiDARMeasurement(3, 1),
                        LiDARMeasurement(2, 1.5),
                        LiDARMeasurement(1, 2),
                        LiDARMeasurement(5, 2.5),
                        LiDARMeasurement(1, 3)]

        p1 = Particle(Pose3D(0, 0, 0))
        p2 = Particle(Pose3D(1, 1, 1))
        p3 = Particle(Pose3D(3, 3, 1.5))
        p4 = Particle(Pose3D(1, 4, 2))
        p5 = Particle(Pose3D(5, 1, 3))

        cost_map = TestMonteCarloLocalization.get_grid_cost_map(20, 30, 20, 30)

        mcl = MonteCarloLocalization()
        mcl.particles = [p1, p2, p3, p4, p5]
        mcl.nb_particles = len(mcl.particles)

        mcl.evaluate_particles(cost_map, measurements)

        weight_sum = 0
        best_particle = mcl.particles[0]
        max_weight = mcl.particles[0].weight
        for p in mcl.particles:
            weight_sum += p.weight
            if best_particle.weight < p.weight:
                best_particle = p
                max_weight = p.weight
        self.assertAlmostEqual(weight_sum, 1, 3)
        self.assertEqual(mcl.id_best_particle, 1)
        self.assertEqual(best_particle, p2)
        self.assertEqual(max_weight, p2.weight)

        print("[OK] Evaluate particles")

    def test_re_sampling(self):
        old_mcl = MonteCarloLocalization()
        old_mcl.particles = []
        old_mcl.particles.append(Particle(Pose3D(0, 0, 0), 0.10))
        old_mcl.particles.append(Particle(Pose3D(0, 10, 0), 0.01))
        old_mcl.particles.append(Particle(Pose3D(0, 20, 0), 0.05))
        old_mcl.particles.append(Particle(Pose3D(0, 30, 0), 0.001))
        old_mcl.particles.append(Particle(Pose3D(10, 0, 1), 0.20))
        old_mcl.particles.append(Particle(Pose3D(10, 10, 1), 0.10))
        old_mcl.particles.append(Particle(Pose3D(10, 20, 1), 0.039))
        old_mcl.particles.append(Particle(Pose3D(10, 30, 1), 0.3))
        old_mcl.particles.append(Particle(Pose3D(20, 0, 2), 0.05))
        old_mcl.particles.append(Particle(Pose3D(20, 10, 2), 0.15))
        old_mcl.nb_particles = len(old_mcl.particles)
        old_mcl.id_best_particle = 7

        bounds = [
            [1, 2],
            [0, 1],
            [0, 1],
            [0, 1],
            [2, 3],
            [1, 2],
            [0, 1],
            [3, 4],
            [0, 1],
            [1, 2],
        ]
        for _ in range(0, 100):

            mcl = copy.deepcopy(old_mcl)

            mcl.re_sampling(0, 0)

            self.assertEqual(len(mcl.particles), len(old_mcl.particles))

            resampled_over = list()
            for i_old, p_old in enumerate(old_mcl.particles):
                resampled_over.append(0)
                for p_new in mcl.particles:
                    if p_old.pose == p_new.pose:
                        resampled_over[i_old] += 1

            for i, nb in enumerate(resampled_over):
                self.assertTrue(bounds[i][0] <= nb <= bounds[i][1])

            self.assertTrue(mcl.particles[mcl.id_best_particle].pose == old_mcl.particles[old_mcl.id_best_particle].pose)

        print("[OK] Resampling")

    def test_estimate_from_odometry(self):
        mcl = MonteCarloLocalization()
        mcl.particles = []
        mcl.particles.append(Particle(Pose3D(1, 1, 0), 0))
        mcl.particles.append(Particle(Pose3D(2, 2, 0.5), 0))
        mcl.particles.append(Particle(Pose3D(3, 3, 1), 0))
        mcl.particles.append(Particle(Pose3D(4, 4, 1.5), 0))
        mcl.particles.append(Particle(Pose3D(5, 5, 2), 0))
        mcl.particles.append(Particle(Pose3D(6, 6, 2.5), 0))
        mcl.particles.append(Particle(Pose3D(7, 7, 3), 0))
        mcl.particles.append(Particle(Pose3D(8, 8, 3.5), 0))
        mcl.nb_particles = len(mcl.particles)

        solution_1_0 = [
            Pose3D(2.0, 1.0, 0),
            Pose3D(2.8775825618903728, 2.479425538604203, 0.5),
            Pose3D(3.5403023058681398, 3.8414709848078967, 1),
            Pose3D(4.070737201667703, 4.997494986604054, 1.5),
            Pose3D(4.583853163452858, 5.909297426825682, 2),
            Pose3D(5.198856384453066, 6.598472144103956, 2.5),
            Pose3D(6.010007503399555, 7.141120008059867, 3),
            Pose3D(7.0635433127092035, 7.64921677231038, 3.5)
        ]

        mcl.estimate_from_odometry(1, 0)

        for i in range(0, mcl.nb_particles):
            self.assertAlmostEqual(mcl.particles[i].pose.x, solution_1_0[i].x, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.z, solution_1_0[i].z, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.theta, solution_1_0[i].theta, 3)

        mcl.particles = []
        mcl.particles.append(Particle(Pose3D(1, 1, 0), 0))
        mcl.particles.append(Particle(Pose3D(2, 2, 0.5), 0))
        mcl.particles.append(Particle(Pose3D(3, 3, 1), 0))
        mcl.particles.append(Particle(Pose3D(4, 4, 1.5), 0))
        mcl.particles.append(Particle(Pose3D(5, 5, 2), 0))
        mcl.particles.append(Particle(Pose3D(6, 6, 2.5), 0))
        mcl.particles.append(Particle(Pose3D(7, 7, 3), 0))
        mcl.particles.append(Particle(Pose3D(8, 8, 3.5), 0))
        mcl.nb_particles = len(mcl.particles)

        solution_0_1 = [
            Pose3D(1.0, 1.0, 1),
            Pose3D(2.0, 2.0, 1.5),
            Pose3D(3.0, 3.0, 2),
            Pose3D(4.0, 4.0, 2.5),
            Pose3D(5.0, 5.0, 3),
            Pose3D(6.0, 6.0, 3.5),
            Pose3D(7.0, 7.0, 4),
            Pose3D(8.0, 8.0, 4.5)
        ]

        mcl.estimate_from_odometry(0, 1)

        for i in range(0, mcl.nb_particles):
            self.assertAlmostEqual(mcl.particles[i].pose.x, solution_0_1[i].x, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.z, solution_0_1[i].z, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.theta, solution_0_1[i].theta, 3)

        mcl.particles = []
        mcl.particles.append(Particle(Pose3D(1, 1, 0), 0))
        mcl.particles.append(Particle(Pose3D(2, 2, 0.5), 0))
        mcl.particles.append(Particle(Pose3D(3, 3, 1), 0))
        mcl.particles.append(Particle(Pose3D(4, 4, 1.5), 0))
        mcl.particles.append(Particle(Pose3D(5, 5, 2), 0))
        mcl.particles.append(Particle(Pose3D(6, 6, 2.5), 0))
        mcl.particles.append(Particle(Pose3D(7, 7, 3), 0))
        mcl.particles.append(Particle(Pose3D(8, 8, 3.5), 0))
        mcl.nb_particles = len(mcl.particles)

        solution_2_3 = [
            Pose3D(3.0, 1.0, 3),
            Pose3D(3.7551651237807455, 2.958851077208406, 3.5),
            Pose3D(4.0806046117362795, 4.6829419696157935, 4),
            Pose3D(4.141474403335406, 5.994989973208109, 4.5),
            Pose3D(4.167706326905715, 6.818594853651364, 5),
            Pose3D(4.397712768906133, 7.196944288207913, 5.5),
            Pose3D(5.02001500679911, 7.282240016119735, 6),
            Pose3D(6.127086625418407, 7.29843354462076, 6.5)
        ]

        mcl.estimate_from_odometry(2, 3)

        for i in range(0, mcl.nb_particles):
            self.assertAlmostEqual(mcl.particles[i].pose.x, solution_2_3[i].x, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.z, solution_2_3[i].z, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.theta, solution_2_3[i].theta, 3)

        mcl.particles = []
        mcl.particles.append(Particle(Pose3D(1, 1, 0), 0))
        mcl.particles.append(Particle(Pose3D(2, 2, 0.5), 0))
        mcl.particles.append(Particle(Pose3D(3, 3, 1), 0))
        mcl.particles.append(Particle(Pose3D(4, 4, 1.5), 0))
        mcl.particles.append(Particle(Pose3D(5, 5, 2), 0))
        mcl.particles.append(Particle(Pose3D(6, 6, 2.5), 0))
        mcl.particles.append(Particle(Pose3D(7, 7, 3), 0))
        mcl.particles.append(Particle(Pose3D(8, 8, 3.5), 0))
        mcl.nb_particles = len(mcl.particles)

        solution_2_4 = [
            Pose3D(-3.5, 1.0, -2.3),
            Pose3D(-1.9491215285066774, -0.15741492371891352, -1.7999999999999998),
            Pose3D(0.5686396235933708, -0.7866194316355344, -1.2999999999999998),
            Pose3D(3.6816825924953367, -0.48872743971824484, -0.7999999999999998),
            Pose3D(6.872660764462141, 0.9081615792844326, -0.2999999999999998),
            Pose3D(9.605146269961201, 3.3068753515321956, 0.20000000000000018),
            Pose3D(11.454966234702004, 6.364959963730597, 0.7000000000000002),
            Pose3D(12.214055092808582, 9.578524524603289, 1.2000000000000002)
        ]

        mcl.estimate_from_odometry(-4.5, -2.3)

        for i in range(0, mcl.nb_particles):
            self.assertAlmostEqual(mcl.particles[i].pose.x, solution_2_4[i].x, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.z, solution_2_4[i].z, 3)
            self.assertAlmostEqual(mcl.particles[i].pose.theta, solution_2_4[i].theta, 3)

        print("[OK] Estimate from odometry")

    def test_add_random_particles(self):

        cost_map = TestMonteCarloLocalization.get_grid_cost_map(20, 30, 20, 30)

        mcl = MonteCarloLocalization()

        mcl.particles = []
        percent_random = 50  # percent random particles
        mcl.nb_particles = 0

        for i in range(len(cost_map.cells)):
            for j in range(len(cost_map.cells[i])):
                if cost_map.cells[i][j].cost != 0:
                    mcl.particles.append(Particle(Pose3D(j*cost_map.size_x+cost_map.size_x/2,
                                                         i*cost_map.size_z+cost_map.size_z/2,
                                                         0),
                                                  0))
                    mcl.nb_particles += 1

        old_mcl = copy.deepcopy(mcl)

        mcl.add_random_particles(cost_map, percent_random)  # 50 : percent

        # test the total number of particles (should be the same as before)
        self.assertEqual(len(mcl.particles), len(old_mcl.particles))

        # test if the particles are in the map, in an obstacle free space
        for i in range(mcl.nb_particles):
            id_x = int(mcl.particles[i].pose.x / cost_map.size_x)
            id_z = int(mcl.particles[i].pose.z / cost_map.size_z)
            self.assertGreaterEqual(id_x, 0)
            self.assertGreaterEqual(id_z, 0)
            self.assertLessEqual(id_x, cost_map.nb_cell_x)
            self.assertLessEqual(id_z, cost_map.nb_cell_z)
            self.assertNotEqual(cost_map.cells[id_z][id_x].cost, 0)
            self.assertNotEqual(cost_map.cells[id_z][id_x].cost, 0)

        nb_different_particles = 0
        for i in range(mcl.nb_particles):
            if mcl.particles[i] != old_mcl.particles[i]:
                nb_different_particles += 1
        self.assertAlmostEqual(nb_different_particles, percent_random*mcl.nb_particles/100, 0)

        print("[OK] Add random particles")
