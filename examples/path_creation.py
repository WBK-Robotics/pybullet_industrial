""" Simple example showcasing the creation of a path
    using various interpolation and path builder functions.
"""

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":

    p.connect(p.GUI)
    # disable rendering during path drawing for better performance
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # creates a blue box path with rounded corners
    test_path = pi.build_box_path(
        [0, 3, 0], [4, 2], 0.7, p.getQuaternionFromEuler([np.pi/2, 0, 0]), 100)
    test_path.draw()
    # creates a blue circle inwin the blue path
    test_path = pi.build_box_path(
        [0, 3, 0], [2, 2], 1, p.getQuaternionFromEuler([np.pi/2, 0, 0]), 100)
    test_path.draw()

    # path in the ++ quadrant
    org_test_path = pi.linear_interpolation(
        [0, 0, 0], [1/np.sqrt(2), 1/np.sqrt(2), 0], 10)
    org_test_path.draw(color=[1, 0, 0])

    test_path = pi.circular_interpolation(
        np.array([0, 1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[1, 0, 0])

    # paths in the +- quadrant
    test_path = pi.linear_interpolation(
        [1, -1, 0], [1-1/np.sqrt(2), -1+1/np.sqrt(2), 0], 10)
    test_path.draw(color=[0, 1, 0])

    test_path = pi.circular_interpolation(
        np.array([0, -1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[0, 1, 0])

    # path in the -- quadrant
    test_path = pi.circular_interpolation(
        np.array([0, -1, 0]), np.array([-1, 0, 0.5]), 1, 50)
    test_path.draw(color=[0, 0, 1])

    # path in the -+ quadrant
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
