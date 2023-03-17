
"""Example demonstraining how the paint particle can be used to color an object.
   Either wait for the robot to coat the upper side of the cube,
   or move it around using the mouse to coat the other sides.

   Carefull this simulation can be compute intensive.
"""

import os

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'kuka_robot.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("cube.urdf", [1.7, 0, 0.0], p.getQuaternionFromEuler(
        [np.pi/2, 0, np.pi/2]), useFixedBase=False)

    robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])

    extruder_properties = {'maximum distance': 0.7,
                           'opening angle': np.pi/2,
                           'material': pi.Paint,
                           'number of rays': 6,
                           'material properties': {'particle size': 0.015,
                                                   'color': [0, 0, 1, 1]}}
    extruder = pi.Extruder(
        urdf_file2, [1.9, 0, 1.2], [0, 0, 0, 1], extruder_properties)
    extruder.couple(robot, 'link6')

    steps = 500
    test_path_2 = pi.linear_interpolation(
        [1.9-0.5, -0.4, 0.7], [1.9+0.5, -0.4, 0.7], steps)

    start_position, start_orientation = test_path_2.get_start_pose()
    for i in range(20):
        extruder.set_tool_pose(start_position, start_orientation)
        for _ in range(50):
            p.stepSimulation()

    while True:
        for positions, orientations, _ in test_path_2:
            extruder.set_tool_pose(positions, orientations)
            position, orientation = extruder.get_tool_pose()
            extruder.extrude()
            p.stepSimulation()

        test_path_2.translate([0, 0.25, 0])

        start_position, start_orientation = test_path_2.get_start_pose()
        for i in range(10):
            extruder.set_tool_pose(start_position, start_orientation)
            for _ in range(50):
                p.stepSimulation()
