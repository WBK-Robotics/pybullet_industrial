
""" This example demonstrates how the camera sensor can be used to capture images.
    Saving the images however requires a external library such as matplotlib
"""

import os
import sys

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'kuka_robot.urdf')
    urdf_file2 = os.path.join(dirname, 'robot_descriptions', 'camera.urdf')
    urdf_file3 = os.path.join(
        dirname, 'robot_descriptions', '3d_printing_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    for i in range(7):
        p.changeVisualShape(
            robot.urdf, -1+i, rgbaColor=[20/255, 68/255, 102/255, 1])

    start_orientation = p.getQuaternionFromEuler([np.pi/2, 0, -0.2*np.pi])
    camera_parameters = {'width': 2272, 'height': 1704, 'fov': 60,
                         'aspect ratio': 1, 'near plane distance': 0.01, 'far plane distance': 100}
    camera = pi.Camera(urdf_file2, [2, 2, 1.1],
                       start_orientation, camera_parameters)

    tool = pi.EndeffectorTool(urdf_file3, [0, 0, 0], start_orientation)
    p.changeVisualShape(
        tool.urdf, -1, rgbaColor=[238/255, 183/255, 13/255, 1])
    tool.couple(robot)

    target_position = [1.0, 0, 0.7]

    for _ in range(200):
        tool.set_tool_pose(target_position, [0, 0, 0, 1])
        p.stepSimulation()

    img = camera.get_image()

    if 'matplotlib' in sys.modules:
        import matplotlib.image
        matplotlib.image.imsave('kuka_robot.png', img)
    else:
        print('Matplotlib not installed, cannot save image')
        print('To install matplotlib, run: pip install matplotlib')

    while True:
        p.stepSimulation()
