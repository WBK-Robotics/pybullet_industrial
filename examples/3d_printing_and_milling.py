"""Example of 3D printing and milling with pybullet_industrial.

This example shows how to use the pybullet_industrial package to simulate two different manufacturing
processes: 3D printing and milling. The example uses a KUKA KR 6 R900 six-axis robot with two different tools:
an extruder and a remover. The extruder is used to print a single layer of a 3D object, while the remover
is used to remove the printed layer.
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
                              'robot_descriptions', '3d_printing_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    fofa_path = os.path.join(dirname,
                             'Objects', 'FoFa', 'FoFa.urdf')
    p.loadURDF(fofa_path, [-4, 5, 0], useFixedBase=True, globalScaling=0.001)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    extruder_properties = {'maximum distance': 0.5,
                           'opening angle': 0,
                           'material': pi.Plastic,
                           'material properties': {'particle size': 0.03},
                           'number of rays': 1}
    extruder = pi.Extruder(
        urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
    p.changeVisualShape(extruder.urdf, -1, rgbaColor=[1, 0, 0, 1])
    extruder.couple(robot, 'link6')

    remover_properties = {'maximum distance': 0.02,
                          'opening angle': 0,
                          'number of rays': 1}
    remover = pi.Remover(
        urdf_file2, [1.9, 1, 1.2], start_orientation, remover_properties)
    p.changeVisualShape(remover.urdf, -1, rgbaColor=[0, 0, 1, 1])

    # Defining a roundet rectangular path
    target_position = np.array([1.9, 0.0, 0.53])
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    test_path = pi.build_box_path(
        target_position, [0.5, 0.6], 0.1, [0, 0, 0, 1], 50)

    for _ in range(20):
        extruder.set_tool_pose(*test_path.get_start_pose())
        for _ in range(50):
            p.stepSimulation()
    p.loadURDF("cube.urdf", [1.9, 0, 0], useFixedBase=True)

    test_path.draw()

    extruding = 1
    while True:
        for _ in range(20):
            extruder.set_tool_pose(*test_path.get_start_pose())
            for _ in range(50):
                p.stepSimulation()
        if extruding:
            for positions, orientations, tool_path in test_path:
                extruder.set_tool_pose(positions, orientations)
                particle = extruder.extrude()

                for _ in range(30):
                    p.stepSimulation()
            extruder.decouple()
            remover.couple(robot, 'link6')
            extruding = 0
            continue
        if not extruding:
            for positions, orientations, tool_path in test_path:
                for _ in range(3):
                    removed_particles = remover.remove()
                remover.set_tool_pose(positions, orientations)
                for _ in range(3):
                    removed_particles = remover.remove()

                for _ in range(30):
                    p.stepSimulation()
            remover.decouple()
            extruder.couple(robot, 'link6')
            extruding = 1
