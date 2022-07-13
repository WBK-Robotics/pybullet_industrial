import pybullet as p
import pybullet_industrial as pi
import numpy as np


def build_circular_path(center, radius, min_angle, max_angle, step_num):
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
    circular_path = np.zeros((3, step_num))
    circular_path[2, :] = center[2]
    for j in range(step_num):
        path_angle = min_angle+j*(max_angle-min_angle)/step_num
        new_position = center[:2] + radius * \
            np.array([-np.cos(path_angle), np.sin(path_angle)])
        circular_path[:2, j] = new_position
    return circular_path


def linear_interpolation(start_point, end_point, samples):
    final_path = np.linspace(start_point, end_point, num=samples)
    return final_path.transpose()


def circular_interpolation(start_point, end_point, radius, samples, clockwise=True):
    connecting_line = end_point-start_point
    distance_between_points = np.linalg.norm(connecting_line)
    if radius <= distance_between_points/2:
        raise ValueError("The radius needs to be at least " +
                         str(distance_between_points/2))
    if start_point[2] != end_point[2]:
        raise ValueError("For circular interpolation both points need to have the same height." +
                         " For height differences use helical interpolation")

    center_distance_from_connecting_line = np.sqrt(
        radius**2-distance_between_points**2/4)

    if clockwise:
        orthogonal_vector = np.array(
            [connecting_line[1], -1*connecting_line[0], connecting_line[2]])
    else:
        orthogonal_vector = np.array(
            [-1*connecting_line[1], connecting_line[0], connecting_line[2]])

    circle_center = start_point+connecting_line/2+center_distance_from_connecting_line * \
        orthogonal_vector/np.linalg.norm(orthogonal_vector)
    print(circle_center, center_distance_from_connecting_line *
          orthogonal_vector/np.linalg.norm(orthogonal_vector))

    angle_range = np.arccos(center_distance_from_connecting_line/radius)*2
    initial_angle = np.arctan2(
        start_point[1]-circle_center[1], start_point[0]-circle_center[0])
    return build_circular_path(circle_center, radius, initial_angle, initial_angle+angle_range, samples)


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
        [0, 0, 0], [2/np.sqrt(1), -2/np.sqrt(1), 0], 10)
    pi.draw_path(test_path, color=[0, 1, 0])

    test_path = circular_interpolation(
        np.array([0, -1, 0]), np.array([1, 0, 0]), 2, 50)
    pi.draw_path(test_path, color=[0, 1, 0])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        p.stepSimulation()
