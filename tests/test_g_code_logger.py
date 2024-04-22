import os
import tempfile
import unittest
from your_module import YourClass  # Import YourClass from your module


def run_simulation(iterator: pi.GCodeProcessor):

    for _ in iterator:
        for _ in range(200):
            p.stepSimulation()


def check_robot_position(robot: pi.RobotBase, target_position,
                         target_orientation,  pos_tol, ori_tol):

    within_precision = True

    current_position, current_orientation = robot.get_endeffector_pose()
    current_orientation = np.array(
        p.getEulerFromQuaternion(current_orientation))

    position_error = np.linalg.norm(current_position-target_position)
    orientation_error = np.linalg.norm(current_orientation -
                                       target_orientation)

    within_precision = within_precision and (
        position_error <= pos_tol) and (orientation_error <= ori_tol)

    return within_precision


class Test_GCodeLogger(unittest.TestCase):

    def test_write_g_code(self):
        # Sample G-code commands
        g_code = [
            {'G': 1, 'X': 10, 'Y': 20, 'Z': 30, 'RA1': 0.5, 'RA2': 1.2},
            {'G': 2, 'X': 15, 'Y': 25, 'Z': 35, 'RA3': 0.8, 'RA4': 1.5},
            {'G': 3, 'X': 20, 'Y': 30, 'Z': 40, 'RA5': 1.0, 'RA6': 1.8}
        ]

        # Create a temporary text file
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file_path = temp_file.name

        # Write G-code to the temporary text file
        YourClass.write_g_code(g_code, temp_file_path)

        # Check if the file was created and contains the expected content
        self.assertTrue(os.path.isfile(temp_file_path))
        with open(temp_file_path, 'r') as file:
            written_content = file.readlines()
            expected_content = [
                'G1 X10 Y20 Z30 RA1:0.5 RA2:1.2\n',
                'G2 X15 Y25 Z35 RA3:0.8 RA4:1.5\n',
                'G3 X20 Y30 Z40 RA5:1.0 RA6:1.8\n'
            ]
            self.assertEqual(written_content, expected_content)

        # Delete the temporary file
        os.remove(temp_file_path)


if __name__ == '__main__':
    unittest.main()
