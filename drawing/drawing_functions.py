# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file contains useful functions to draw on a Tkinter canvas 
"""

from math import cos, sin
from geometry.point import Point2D
from geometry.segment import Segment2D


def x_real_2_draw(canvas, x_real, world_dimension):
    """
        function to convert an x value from the world frame to the display frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            x_real: (number in m) the x value in the world frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        RETURNS:
            (number) the corresponding x value in the canvas frame

    """
    return x_real * canvas.winfo_width() / world_dimension.width


def z_real_2_draw(canvas, z_real, world_dimension):
    """
        function to convert a z value from the world frame to the display frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            z_real: (number in m) the z value in the world frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        RETURNS:
            (number) the corresponding z value in the canvas frame
    """
    return z_real * canvas.winfo_height() / world_dimension.height


def x_draw_2_real(canvas, x_draw, world_dimension):
    """
        function to convert an x value from the display frame to the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            x_draw: (number) the x value in the display frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        RETURNS:
            (number) the corresponding x value in the world frame
    """
    return x_draw * world_dimension.width / canvas.winfo_width()


def z_draw_2_real(canvas, z_draw, world_dimension):
    """
        function to convert a z value from the display frame to the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            z_draw: (number) the z value in the display frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        RETURNS:
            (number) the corresponding z value in the world frame
    """
    return z_draw * world_dimension.height / canvas.winfo_height()


def draw_robot(canvas, robot, world_dimension):
    """
        function to draw a robot
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            robot: (robot.Robot) the robot to draw
            world_dimension: (should have width and height attributes) the world the robot is in
    """

    """ ***** Drawing the body of the robot (circle) ***** """
    x_left_top = x_real_2_draw(canvas, robot.pose.x - robot.wheel_distance, world_dimension)
    z_left_top = z_real_2_draw(canvas, robot.pose.z - robot.wheel_distance, world_dimension)

    x_right_bot = x_real_2_draw(canvas, robot.pose.x + robot.wheel_distance, world_dimension)
    z_right_bot = z_real_2_draw(canvas, robot.pose.z + robot.wheel_distance, world_dimension)

    canvas.create_oval(x_left_top, z_left_top, x_right_bot, z_right_bot, fill='grey', outline='black')

    """ ***** Drawing the arrow for the robot direction ***** """
    x_start = x_real_2_draw(canvas, robot.pose.x, world_dimension)
    z_start = z_real_2_draw(canvas, robot.pose.z, world_dimension)
    x_end = x_real_2_draw(canvas, robot.pose.x + cos(robot.pose.theta) * robot.wheel_distance, world_dimension)
    z_end = z_real_2_draw(canvas, robot.pose.z + sin(robot.pose.theta) * robot.wheel_distance, world_dimension)
    canvas.create_line(x_start, z_start, x_end, z_end, fill='white', arrow='last')


def draw_grid_map(canvas, grid_map):
    """
        function to draw a grid map
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            grid_map: (environment.GridMap) the grid map to draw
    """
    # loops over all the cells of the grid map
    for z in grid_map.cells:
        for c in z:
            color = '#%02x%02x%02x' % (int(255 - c.val * 255), int(255 - c.val * 255), int(255 - c.val * 255))
            canvas.create_rectangle(x_real_2_draw(canvas, c.x * grid_map.size_x, grid_map),
                                    z_real_2_draw(canvas, c.z * grid_map.size_z, grid_map),
                                    x_real_2_draw(canvas, c.x * grid_map.size_x + grid_map.size_x, grid_map),
                                    z_real_2_draw(canvas, c.z * grid_map.size_z + grid_map.size_z, grid_map),
                                    fill=color, outline="white")


def draw_seg_map(canvas, seg_map):
    """
        function to draw a seg map
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            seg_map: (environment.seg_map.SegMap) the segments map to draw
    """
    for s in seg_map.obstaclesSeg:
        draw_segment(canvas, s, "blue", 2, seg_map)


def draw_cost_map(canvas, cost_map):
    """
        function to draw a cost map
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            cost_map: (environment.CostMap) the cost map to draw
    """
    if len(cost_map.cells) > 0 and cost_map.max_cost > 0:
        norm = 1.0 / cost_map.max_cost
        for z in cost_map.cells:
            for c in z:
                color = '#FFFF00'
                if c.cost != float('inf'):
                    color = '#%02x%02x%02x' % (int(255 - c.cost * norm * 255), int(255 - c.cost * norm * 255),
                                               int(255 - c.cost * norm * 255))
                canvas.create_rectangle(x_real_2_draw(canvas, c.x * cost_map.size_x, cost_map),
                                        z_real_2_draw(canvas, c.z * cost_map.size_z, cost_map),
                                        x_real_2_draw(canvas, c.x * cost_map.size_x + cost_map.size_x, cost_map),
                                        z_real_2_draw(canvas, c.z * cost_map.size_z + cost_map.size_z, cost_map),
                                        fill=color, outline="white")


def draw_exploration_map(canvas, exploration_map):
    """
        function to draw an exploration map
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            exploration_map: (environment.CostMap) the cost map to draw
    """
    for z in exploration_map.cells:
        for c in z:
            if c.is_frontier is True:
                color = "orange"
            elif c.cost == float('inf'):
                if c.val == 0.5:
                    tmp = 200
                else:
                    tmp = int(255 - c.val * 255)
                color = '#%02x%02x%02x' % (tmp, tmp, tmp)

            else:
                tmp = int(255 - c.cost * 200 / exploration_map.max_cost)
                color = '#%02x%02x%02x' % (tmp, tmp, 255)

            canvas.create_rectangle(x_real_2_draw(canvas, c.x * exploration_map.size_x, exploration_map),
                                    z_real_2_draw(canvas, c.z * exploration_map.size_z, exploration_map),
                                    x_real_2_draw(canvas, c.x * exploration_map.size_x + exploration_map.size_x,
                                                  exploration_map),
                                    z_real_2_draw(canvas, c.z * exploration_map.size_z + exploration_map.size_z,
                                                  exploration_map),
                                    fill=color, outline="white")


def draw_segment(canvas, segment, color, width, world_dimension):
    """
        function to draw a segment with coordinates in the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            segment: (geometry.Segment) the segment to draw
            color: (string) the color to draw the segment
            width: (number) the size to draw the segment with
            world_dimension: (should have width and height attributes) the world the segment is defined in
    """
    canvas.create_line(x_real_2_draw(canvas, segment.p1.x, world_dimension),
                       z_real_2_draw(canvas, segment.p1.z, world_dimension),
                       x_real_2_draw(canvas, segment.p2.x, world_dimension),
                       z_real_2_draw(canvas, segment.p2.z, world_dimension),
                       fill=color, width=width)


def draw_seg_environment(canvas, seg_environment, color='red'):
    """
        function to draw a segment with coordinates in the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            seg_environment: (environment.SegEnv) the segment environment (list of segments) to draw
            color: (string) the color to draw the environment with
    """
    for seg in seg_environment.segments:
        draw_segment(canvas, seg, color, 3, seg_environment)


def draw_lidar_measurements(canvas, measurements, pose, world_dimension):
    """
        function to draw a set of LiDAR Measurements, according to the pose (x, z, theta) in the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            measurements: (list of LidarMeasurement) the LiDAR measurements to draw
            pose: (robot.pose.Pose3D) the pose for the measurements
            world_dimension: (should have width and height attributes) the world the measurements are defined in
    """
    if measurements is not None:
        for msr in measurements:
            point = Point2D(cos(msr.angle + pose.theta) * msr.distance + pose.x,
                            sin(msr.angle + pose.theta) * msr.distance + pose.z)
            canvas.create_line(x_real_2_draw(canvas, pose.x, world_dimension),
                               z_real_2_draw(canvas, pose.z, world_dimension),
                               x_real_2_draw(canvas, point.x, world_dimension),
                               z_real_2_draw(canvas, point.z, world_dimension), fill="red", width=2)


def draw_point(canvas, point, size, color, world_dimension):
    """
        function to draw a point with coordinates in the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            point: (geometry.point Point2D) the point to draw
            size: (number) the size of the point to draw
            color: (string) the color to draw the point
            world_dimension: (should have width and height attributes) the world the point is defined in
    """
    x_left_top = x_real_2_draw(canvas, point.x - size, world_dimension)
    z_left_top = z_real_2_draw(canvas, point.z - size, world_dimension)
    x_right_bot = x_real_2_draw(canvas, point.x + size, world_dimension)
    z_right_bot = z_real_2_draw(canvas, point.z + size, world_dimension)
    canvas.create_oval(x_left_top, z_left_top, x_right_bot, z_right_bot, fill=color, outline=color)


def draw_points(canvas, points, size, color, world_dimension):
    """
        function to draw a list of points with coordinates in the canvas frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            points: (list of geometry.point Point2D) the points we want to draw
            color: (string) the color to draw the points
            size: (string) the color to draw the point
            world_dimension: (string) the color to draw the point
    """
    for point in points:
        draw_point(canvas, point, size, color, world_dimension)


def draw_associations(canvas, model, scene, associations, world_dimension):
    """
    function to draw point associations
    PARAMETERS:
        canvas: (tkinter.Canvas) the display
        model: (list of geometry.point.Point2D) the points of the model
        scene: (list of geometry.point.Point2D) the points of the scene
        associations: (list of tp_icp.association.Association) The associations to draw
        world_dimension: (should have width and height attributes) the world the measurements are defined in
    """
    if len(associations) == len(scene):
        for association in associations:
            segment = Segment2D(scene[association.id_scene], model[association.id_model])
            draw_segment(canvas, segment, "orange", 2, world_dimension)


def draw_particles(canvas, robot, mcl, world_dimension):
    """
        function to draw a set particles with pose in the world frame
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            robot: (robot.robot.Robot) the robot the particles are evaluated the pose of
            mcl: (tp_mcl.monte_carlo MonteCarloLocalization) The MonteCarloLocalization object we want to draw
            the particle of
            world_dimension: (should have width and height attributes) the world the measurements are defined in
    """
    for particle in mcl.particles:
        color = '#%02x%02x%02x' % (100, 100, int(particle.weight * 255 / mcl.max_weight))

        x1 = x_real_2_draw(canvas, particle.pose.x, world_dimension)
        z1 = z_real_2_draw(canvas, particle.pose.z, world_dimension)

        xh = particle.pose.x + cos(particle.pose.theta) * robot.wheel_distance
        zh = particle.pose.z + sin(particle.pose.theta) * robot.wheel_distance
        xh1, zh1 = x_real_2_draw(canvas, xh, world_dimension), z_real_2_draw(canvas, zh, world_dimension)

        canvas.create_line(x1, z1, xh1, zh1, fill=color, arrow='last')


def draw_rrt(canvas, rrt, color, size, world_dimension):
    for i in range(0, len(rrt.nodes)):
        if i == len(rrt.nodes)-1:
            canvas.create_line(x_real_2_draw(canvas, rrt.nodes[i].x, world_dimension),
                               z_real_2_draw(canvas, rrt.nodes[i].z, world_dimension),
                               x_real_2_draw(canvas, rrt.nodes[rrt.nodes[i].parent].x, world_dimension),
                               z_real_2_draw(canvas, rrt.nodes[rrt.nodes[i].parent].z, world_dimension),
                               fill=color, width=2)
        elif i != 0:
            canvas.create_line(x_real_2_draw(canvas, rrt.nodes[i].x, world_dimension),
                               z_real_2_draw(canvas, rrt.nodes[i].z, world_dimension),
                               x_real_2_draw(canvas, rrt.nodes[rrt.nodes[i].parent].x, world_dimension),
                               z_real_2_draw(canvas, rrt.nodes[rrt.nodes[i].parent].z, world_dimension),
                               fill=color, width=2)

        draw_point(canvas, rrt.nodes[i], size, color, world_dimension)


def draw_path(canvas, rrt, color, size, world_dimension):
    for n in rrt.path:
        draw_point(canvas, n, size, color, world_dimension)


def draw_astar(canvas, astar):
    """
        function to draw an A*
        PARAMETERS:
            canvas: (tkinter.Canvas) the display
            astar: (tp_a_star.astar.AStar) the A* you want to draw
    """
    for z in astar.nodes:
        for c in z:
            color = '#FFFFFF'
            if c.is_path:
                color = '#0000FF'
            elif c.is_closed:
                color = '#FF0000'
            elif c.is_opened:
                color = '#00FF00'
            elif not c.walkable:
                color = '#000000'
            canvas.create_rectangle(x_real_2_draw(canvas, c.position.x * astar.sizeX, astar),
                                    z_real_2_draw(canvas, c.position.z * astar.sizeZ, astar),
                                    x_real_2_draw(canvas, c.position.x * astar.sizeX + astar.sizeX, astar),
                                    z_real_2_draw(canvas, c.position.z * astar.sizeZ + astar.sizeZ, astar),
                                    fill=color, outline="white")
