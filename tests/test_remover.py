import os

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import unittest


dirname = os.path.dirname(__file__)
parentDir = os.path.dirname(dirname)
urdf_file1 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'comau_nj290_robot.urdf')
urdf_file2 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'milling_head.urdf')


class TestRemover(unittest.TestCase):
    def test_dot_removel(self):
        """This test checks that the remover can invert the results of a extruder and remove all
           particles the extruder has previously printed.
        """

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        extruder_properties = {'maximum distance': 0.5,
                               'opening angle': 0,
                               'material': pi.Plastic,
                               'material properties': {'particle size': 0.05},
                               'number of rays': 1}
        extruder = pi.Extruder(
            urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
        extruder.couple(robot, 'link6')

        remover_properties = {'maximum distance': 0.02,
                              'opening angle': 0,
                              'number of rays': 1}
        remover = pi.Remover(
            urdf_file2, [1.9, 1, 1.2], start_orientation, remover_properties)

        target_position = np.array([1.9, 0.0, 1.03])
        target_orientation = p.getQuaternionFromEuler([0, 0, 0])
        steps = 20
        test_path = pi.build_box_path(
            target_position, [0.4, 0.4], 0.2, [0, 0, 0, 1], steps)

        current_particles = []

        for _ in range(20):
            extruder.set_tool_pose(*test_path.get_start_pose())
            for _ in range(50):
                p.stepSimulation()

        for target_position, target_orientation, _ in test_path:
            extruder.set_tool_pose(target_position, target_orientation)
            particle = extruder.extrude()
            current_particles.append(particle[0].particle_id)

            for _ in range(30):
                p.stepSimulation()
        extruder.decouple()
        remover.couple(robot, 'link6')

        for _ in range(20):
            extruder.set_tool_pose(*test_path.get_start_pose())
            for _ in range(50):
                p.stepSimulation()

        for target_position, target_orientation, _ in test_path:
            for _ in range(3):
                removed_particles = remover.remove()
                for elements in removed_particles:
                    current_particles.remove(elements)
            remover.set_tool_pose(target_position, target_orientation)
            for _ in range(3):
                removed_particles = remover.remove()
                for elements in removed_particles:
                    current_particles.remove(elements)

            for _ in range(30):
                p.stepSimulation()

        p.disconnect()
        self.assertTrue(current_particles == [])


if __name__ == '__main__':
    unittest.main()
