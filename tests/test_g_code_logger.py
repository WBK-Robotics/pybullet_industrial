import os
import unittest
import pybullet_industrial as pi
import pybullet as p
import pybullet_data
import numpy as np
import tempfile


class Test_GCodeLogger(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        urdf_robot = os.path.join(
            parentDir, 'examples', 'robot_descriptions',
            'comau_nj290_robot.urdf')

        urdf_fofa = os.path.join(
            parentDir, 'examples', 'Objects', 'FoFa', 'FoFa.urdf')

        # Setting up the simulation
        start_pos = np.array([2.0, -6.5, 0])
        p.connect(p.GUI, options='--background_color_red=1 ' +
                  '--background_color_green=1 ' +
                  '--background_color_blue=1')
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0, cameraYaw=50.0,
            cameraPitch=-30,
            cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

        # Setting up robot position
        cls.robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
        cls.robot.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
        for _ in range(100):
            p.stepSimulation()
        new_point = cls.robot.get_endeffector_pose()[0]
        cls.robot.set_endeffector_pose(
            new_point, p.getQuaternionFromEuler([0, 0, 0]))
        for _ in range(100):
            p.stepSimulation()

    def test_write_g_code(self):
        # Create Test Object using the robot object created in setUpClass
        test_object = pi.GCodeLogger(self.robot)

        # Sample G-code commands
        g_code = [
            {'G': 0, 'X': 10, 'Y': 20, 'Z': 30, 'A': 0, 'B': 0, 'C': 0},
            {'G': 1, 'X': 15, 'Y': 25, 'Z': 35, 'A': 0, 'B': 0, 'C': 0},
            {'G': 1, 'RA1': 0.5, 'RA2': 1.2, 'RA3': 0.8,
             'RA4': 1.5, 'RA5': 1.0, 'RA6': 1.8}
        ]

        # Create a temporary text file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file_path = temp_file.name

        # Write G-code to the temporary text file
        test_object.write_g_code(g_code, temp_file_path)

        # Check if the file was created and contains the expected content
        self.assertTrue(os.path.isfile(temp_file_path))
        with open(temp_file_path, 'r') as file:
            written_content = file.readlines()
            expected_content = [
                'G0 X10 Y20 Z30 A0 B0 C0\n',
                'G1 X15 Y25 Z35 A0 B0 C0\n',
                'G1 RA1=0.5 RA2=1.2 RA3=0.8 RA4=1.5 RA5=1.0 RA6=1.8\n'
            ]
            self.assertEqual(written_content, expected_content)

        # Delete the temporary file
        os.remove(temp_file_path)


if __name__ == '__main__':
    unittest.main()
