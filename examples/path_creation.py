import pybullet as p
import pybullet_industrial as pi
import numpy as np
import scipy.interpolate as sci


def build_circular_path(center, radius, min_angle, max_angle, step_num, clockwise=True):
    """Function which builds a circular path

    Args:
        center (array): the center of the circle
        radius (float): the radius of the circle
        min_angle (float): minimum angle of the circle path
        max_angle (float): maximum angle of the circle path
        steps (int): the number of steps between min_angle and max_angle

    Returns:
        array: array of 3 dimensional path points
    """

    circular_path = np.zeros((2, step_num))
    for j in range(step_num):
        if clockwise:
            path_angle = min_angle-j*(max_angle-min_angle)/step_num
        else:
            path_angle = min_angle+j*(max_angle-min_angle)/step_num
        new_position = center + radius * \
            np.array([np.cos(path_angle), np.sin(path_angle)])
        circular_path[:, j] = new_position
    return circular_path


def linear_interpolation(start_point, end_point, samples):
    final_path = np.linspace(start_point, end_point, num=samples)
    return final_path.transpose()


def planar_circular_interpolation(start_point, end_point, radius, samples, clockwise=True):
    connecting_line = end_point-start_point
    distance_between_points = np.linalg.norm(connecting_line)
    if radius <= distance_between_points/2:
        raise ValueError("The radius needs to be at least " +
                         str(distance_between_points/2))

    center_distance_from_connecting_line = np.sqrt(
        radius**2-distance_between_points**2/4)

    if clockwise:
        orthogonal_vector = np.array(
            [connecting_line[1], -1*connecting_line[0]])
    else:
        orthogonal_vector = np.array(
            [-1*connecting_line[1], connecting_line[0]])

    circle_center = start_point+connecting_line/2+center_distance_from_connecting_line * \
        orthogonal_vector/np.linalg.norm(orthogonal_vector)

    angle_range = np.arccos(center_distance_from_connecting_line/radius)*2
    initial_angle = np.arctan2(
        start_point[1]-circle_center[1], start_point[0]-circle_center[0])

    planar_path = build_circular_path(
        circle_center, radius, initial_angle, initial_angle+angle_range, samples, clockwise)
    return planar_path


def circular_interpolation(start_point, end_point, radius, samples, axis=2, clockwise=True):
    all_axis = [0, 1, 2]
    all_axis.remove(axis)
    planar_start_point = np.array(
        [start_point[all_axis[0]], start_point[all_axis[1]]])
    planar_end_point = np.array(
        [end_point[all_axis[0]], end_point[all_axis[1]]])

    planar_path = planar_circular_interpolation(
        planar_start_point, planar_end_point, radius, samples, clockwise)

    path = np.zeros((3, samples))
    for i in range(2):
        path[all_axis[i]] = planar_path[i]
    path[axis] = np.linspace(start_point[axis], end_point[axis], samples)
    return path


def spline_interpolation(points, samples):
    s = np.linspace(0, 1, len(points[0]))

    path = np.zeros((3, samples))
    print(points[0], s)
    cs_x = sci.CubicSpline(s, points[0])
    cs_y = sci.CubicSpline(s, points[1])
    cs_z = sci.CubicSpline(s, points[2])

    cs_s = np.linspace(0, 1, samples)
    path[0] = cs_x(cs_s)
    path[1] = cs_y(cs_s)
    path[2] = cs_z(cs_s)

    return path


if __name__ == "__main__":

    p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # ++ quadrant
    test_path = linear_interpolation(
        [0, 0, 0], [1/np.sqrt(2), 1/np.sqrt(2), 0], 10)
    pi.draw_path(test_path, color=[1, 0, 0])

    test_path = circular_interpolation(
        np.array([0, 1, 0]), np.array([1, 0, 0]), 1, 50)
    pi.draw_path(test_path, color=[1, 0, 0])

    # +- quadrant
    test_path = linear_interpolation(
        [1, -1, 0], [1-1/np.sqrt(2), -1+1/np.sqrt(2), 0], 10)
    pi.draw_path(test_path, color=[0, 1, 0])

    test_path = circular_interpolation(
        np.array([0, -1, 0]), np.array([1, 0, 0]), 1, 50)
    pi.draw_path(test_path, color=[0, 1, 0])

    # -- quadrant
    test_path = circular_interpolation(
        np.array([0, -1, 0]), np.array([-1, 0, 0.5]), 1, 50)
    pi.draw_path(test_path, color=[0, 0, 1])

    # -+ quadrant
    test_path = circular_interpolation(
        np.array([0, 1, 0.5]), np.array([-1, 0, 0.5]), 1, 50, clockwise=False)
    pi.draw_path(test_path, color=[0, 1, 1])

    test_path = circular_interpolation(
        np.array([0, 0, 0]), np.array([0, 1, 1]), 1, 50, axis=0)
    pi.draw_path(test_path, color=[1, 1, 0])
    test_path = circular_interpolation(
        np.array([0, 0, 0]), np.array([1, 0, 1]), 1, 50, axis=1)
    pi.draw_path(test_path, color=[1, 1, 0])

    test_path = spline_interpolation(
        [[0, 1, 0], [0, 0, 1], [0, 1, 1]], samples=50)
    pi.draw_path(test_path, color=[1, 0.5, 0.5])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        p.stepSimulation()
