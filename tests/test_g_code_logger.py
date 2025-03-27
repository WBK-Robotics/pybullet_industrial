import os
import unittest
import pybullet_industrial as pi
import pybullet as p
import pybullet_data
import numpy as np
import tempfile


def run_simulation(iterator: pi.GCodeProcessor):
    for _ in iterator:
        for _ in range(200):
            p.stepSimulation()


def round_float_values(g_code, decimals):
    rounded_list = []
    for item in g_code:
        rounded_item = {key: round(value, decimals) if isinstance(
            value, float) else value for key, value in item.items()}
        rounded_list.append(rounded_item)
    return rounded_list


class Test_GCodeLogger(unittest.TestCase):

    def test_write_g_code(self):
        # Sample G-code commands
        g_code = [
            {'G': 0, 'X': 10, 'Y': 20, 'Z': 30, 'A': 0, 'B': 0, 'C': 0},
            {'G': 1, 'X': 15, 'Y': 25, 'Z': 35, 'A': 0, 'B': 0, 'C': 0},
            {'G': 1, 'RA1': 0.5, 'RA2': 1.2, 'RA3': 0.8,
             'RA4': 1.5, 'RA5': 1.0, 'RA6': 1.8},
            {'T': 2, 'M': 5},
            {'M': 11},
            {'RA1': 1e-10}
        ]
        # Create a temporary text file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file_path = temp_file.name
        # Write G-code to the temporary text file
        pi.GCodeLogger.write_g_code(g_code, temp_file_path, {
            'M11': '%@example_call'},
            "%prefix", "%postfix")
        # Check if the file was created and contains the expected content
        self.assertTrue(os.path.isfile(temp_file_path))
        with open(temp_file_path, 'r') as file:
            written_content = file.readlines()
            expected_content = [
                '%prefix\n',
                'G0 X10 Y20 Z30 A0 B0 C0\n',
                'G1 X15 Y25 Z35 A0 B0 C0\n',
                'G1 RA1=0.5 RA2=1.2 RA3=0.8 RA4=1.5 RA5=1 RA6=1.8\n',
                'T2 M5\n',
                '%@example_call\n',
                'RA1=0.0000000001\n',
                '%postfix\n'
            ]
            self.assertEqual(written_content, expected_content)
        # Delete the temporary file
        os.remove(temp_file_path)

        # Write G-Code translate is None
        g_code = [
            {'M': 11}
        ]
        # Create a temporary text file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file_path = temp_file.name
        # Write G-code to the temporary text file
        pi.GCodeLogger.write_g_code(g_code, temp_file_path)
        # Check if the file was created and contains the expected content
        self.assertTrue(os.path.isfile(temp_file_path))
        with open(temp_file_path, 'r') as file:
            written_content = file.readlines()
            expected_content = [
                'M11\n'
            ]
            self.assertEqual(written_content, expected_content)
        # Delete the temporary file
        os.remove(temp_file_path)


    def test_uppdate_g_code(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        urdf_robot = os.path.join(
            parentDir, 'examples', 'robot_descriptions',
            'comau_nj290_robot.urdf')

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=10000)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        start_position = np.array([2.0, -6.5, 0])

        robot = pi.RobotBase(
            urdf_robot, start_position, start_orientation)
        robot.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
        for _ in range(100):
            p.stepSimulation()
        new_point = robot.get_endeffector_pose()[0]
        robot.set_endeffector_pose(
            new_point, p.getQuaternionFromEuler([0, 0, 0]))
        for _ in range(100):
            p.stepSimulation()

        # Create test objects using the robot object created in setUpClass
        g_code_logger = pi.GCodeLogger(robot)
        g_code_processor = pi.GCodeProcessor(
            robot=robot)
        g_code_iterator = iter(g_code_processor)
        # Test G-Code robot view
        input_g_code = [
            {'G': 1, 'X': 4.0, 'Y': -5.5, 'Z': 1.8,
                'A': -1.57, 'B': 0.0, 'C': 0.0},
            {'G': 1, 'X': 4.5, 'Y': -6.0, 'Z': 1.5,
                'A': -0.79, 'B': 0.0, 'C': 0.0},
            {'G': 1, 'X': 4.5, 'Y': -6.0, 'Z': 2.0,
                'A': -0.79, 'B': 0.0, 'C': 0.0},
        ]
        # Run first entry
        g_code_processor.g_code = [input_g_code[0]]
        run_simulation(g_code_iterator)
        g_code_logger.update_g_code_robot_view()
        # Run second entry
        g_code_processor.g_code = [input_g_code[1]]
        run_simulation(g_code_iterator)
        g_code_logger.update_g_code_robot_view()
        # Compare G-Codes
        output_g_code_robot_view = round_float_values(
            g_code_logger.g_code_robot_view, 2)
        self.assertEqual(input_g_code[:2], output_g_code_robot_view)
        # Run second entry
        g_code_processor.g_code = [input_g_code[2]]
        run_simulation(g_code_iterator)
        g_code_logger.update_g_code_robot_view(False)
        output_g_code_robot_view = round_float_values(
            g_code_logger.g_code_robot_view, 2)
        self.assertEqual(input_g_code, output_g_code_robot_view)
        # Test G-Code joint position
        input_g_code = [
            {'G': 1, 'RA1': 0.46, 'RA2': 0.24, 'RA3': -1.55,
                'RA4': -3.14, 'RA5': -1.36, 'RA6': 0.46},
            {'G': 1, 'RA1': 0.12, 'RA2': 0.57, 'RA3': -1.38,
                'RA4': -2.3, 'RA5': -1.22, 'RA6': -0.19}
        ]
        # Run first entry
        g_code_processor.g_code = [input_g_code[0]]
        run_simulation(g_code_iterator)
        g_code_logger.update_g_code_joint_position()
        # Run second entry
        g_code_processor.g_code = [input_g_code[1]]
        run_simulation(g_code_iterator)
        g_code_logger.update_g_code_joint_position()
        # Compare G-Codes
        output_g_code_joint_position = round_float_values(
            g_code_logger.g_code_joint_position, 2)
        p.disconnect()
        self.assertEqual(input_g_code, output_g_code_joint_position)


if __name__ == '__main__':
    unittest.main()
