import pybullet as p
import pybullet_industrial as pi
import numpy as np


def build_box_path(center_position, dimensions, radius, orientation, samples):
    corner_point_00 = np.array(
        [-0.5*dimensions[0], -0.5*dimensions[1]+radius, 0])
    corner_point_01 = np.array(
        [-0.5*dimensions[0], 0.5*dimensions[1]-radius, 0])
    corner_point_10 = np.array(
        [-0.5*dimensions[0]+radius, 0.5*dimensions[1], 0])
    corner_point_11 = np.array(
        [0.5*dimensions[0]-radius, 0.5*dimensions[1], 0])
    corner_point_20 = np.array(
        [0.5*dimensions[0], 0.5*dimensions[1]-radius, 0])
    corner_point_21 = np.array(
        [0.5*dimensions[0], -0.5*dimensions[1]+radius, 0])
    corner_point_30 = np.array(
        [0.5*dimensions[0]-radius, -0.5*dimensions[1], 0])
    corner_point_31 = np.array(
        [-0.5*dimensions[0]+radius, -0.5*dimensions[1], 0])

    linear_length = (np.linalg.norm(corner_point_00-corner_point_01) +
                     np.linalg.norm(corner_point_10-corner_point_11) +
                     np.linalg.norm(corner_point_20-corner_point_21) +
                     np.linalg.norm(corner_point_30-corner_point_31))

    circle_length = 2*np.pi*radius

    linear_samples = int(0.25*linear_length /
                         (linear_length+circle_length)*samples)
    circle_samples = int(0.25*circle_length /
                         (linear_length+circle_length)*samples)

    test_path = pi.linear_interpolation(
        corner_point_00, corner_point_01, linear_samples)
    side_1 = pi.linear_interpolation(
        corner_point_10, corner_point_11, linear_samples)
    side_2 = pi.linear_interpolation(
        corner_point_20, corner_point_21, linear_samples)
    side_3 = pi.linear_interpolation(
        corner_point_30, corner_point_31, linear_samples)

    corner_0 = pi.circular_interpolation(
        corner_point_01, corner_point_10, radius, circle_samples)
    corner_1 = pi.circular_interpolation(
        corner_point_11, corner_point_20, radius, circle_samples)
    corner_2 = pi.circular_interpolation(
        corner_point_21, corner_point_30, radius, circle_samples)
    corner_3 = pi.circular_interpolation(
        corner_point_31, corner_point_00, radius, circle_samples)

    test_path.append(corner_0)
    test_path.append(side_1)
    test_path.append(corner_1)
    test_path.append(side_2)
    test_path.append(corner_2)
    test_path.append(side_3)
    test_path.append(corner_3)

    test_path.rotate(orientation)
    test_path.translate(center_position)

    return test_path


if __name__ == "__main__":

    p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    test_path = build_box_path(
        [0, 3, 0], [4, 2], 0.7, p.getQuaternionFromEuler([np.pi/2, 0, 0]), 100)
    test_path.draw()

    test_path = build_box_path(
        [0, 3, 0], [2, 2], 1, p.getQuaternionFromEuler([np.pi/2, 0, 0]), 100)
    test_path.draw()

    # ++ quadrant
    org_test_path = pi.linear_interpolation(
        [0, 0, 0], [1/np.sqrt(2), 1/np.sqrt(2), 0], 10)
    org_test_path.draw(color=[1, 0, 0])

    test_path = pi.circular_interpolation(
        np.array([0, 1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[1, 0, 0])

    # +- quadrant
    test_path = pi.linear_interpolation(
        [1, -1, 0], [1-1/np.sqrt(2), -1+1/np.sqrt(2), 0], 10)
    test_path.draw(color=[0, 1, 0])

    test_path = pi.circular_interpolation(
        np.array([0, -1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[0, 1, 0])

    # -- quadrant
    test_path = pi.circular_interpolation(
        np.array([0, -1, 0]), np.array([-1, 0, 0.5]), 1, 50)
    test_path.draw(color=[0, 0, 1])

    # -+ quadrant
    test_path = pi.circular_interpolation(
        np.array([0, 1, 0.5]), np.array([-1, 0, 0.5]), 1, 50, clockwise=False)
    test_path.draw(color=[0, 1, 1])

    test_path = pi.circular_interpolation(
        np.array([0, 0, 0]), np.array([0, 1, 1]), 1, 50, axis=0)
    test_path.draw(color=[1, 1, 0])

    test_path = pi.circular_interpolation(
        np.array([0, 0, 0]), np.array([1, 0, 1]), 1, 50, axis=1)
    test_path.draw(color=[1, 1, 0])

    test_path = pi.spline_interpolation(
        [[0, 1, 0], [0, 0, 1], [0, 1, 1]], samples=50)
    test_path.draw(color=[1, 0.5, 0.5])

    test_path.translate([0, 0, 1])
    test_path.draw(pose=True)

    test_path.append(org_test_path)
    test_path.prepend(org_test_path)

    test_path.rotate(p.getQuaternionFromEuler([np.pi/2, 0, 0]))
    test_path.draw(pose=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        p.stepSimulation()
        for position, orientation, _ in test_path:
            print(position)
