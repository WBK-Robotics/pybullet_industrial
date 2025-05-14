import unittest
import numpy as np
from copy import deepcopy
from pybullet_industrial.g_code_simplifier import GCodeSimplifier


class TestGCodeSimplifier(unittest.TestCase):
    """
    Unit tests for GCodeSimplifier methods.
    """

    def setUp(self):
        self.g_code = [
            {'X': 10.1234, 'Y': 20.5678, 'A': 30.9876, 'G': 1},
            {'X': 15.2222, 'Y': 25.9999, 'B': 45.5432},
            {'Z': 5.8765,  'C': 60.6543},
            {'RA1': 90.3333, 'G': 1}
        ]

    def test_round_pose_values(self):
        """
        Test rounding of pose values (X, Y, Z, A, B, C) to given precision.

        Verifies that numeric values are rounded correctly and non-pose
        keys remain unchanged.
        """
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.round_pose_values(g_code, 2, 2)
        self.assertEqual(result[0]['X'], round(10.1234, 2))
        self.assertEqual(result[0]['A'], round(30.9876, 2))
        self.assertEqual(result[1]['X'], round(15.2222, 2))
        self.assertEqual(result[2]['C'], round(60.6543, 2))

    def test_round_joint_positions(self):
        """
        Test rounding of joint positions with keys starting with 'RA'
        followed by digits.

        Ensures correct precision is applied and non-matching keys
        (e.g., 'RAx') are left unchanged.
        """
        g_code = deepcopy(self.g_code)
        g_code.append({'RA2': 45.67891, 'RAx': 50.12345})
        result = GCodeSimplifier.round_joint_positions(g_code, 3)
        self.assertEqual(result[3]['RA1'], round(90.3333, 3))
        self.assertEqual(result[4]['RA2'], round(45.67891, 3))
        self.assertEqual(result[4]['RAx'], 50.12345)

    def test_scale_g_code(self):
        """
        Test scaling of specified G-code keys by a given factor.

        Verifies that only the targeted keys are scaled and others remain
        unaffected.
        """
        scaling = 2.0
        keys = ['X', 'Y']
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.scale_g_code(g_code, scaling, keys)
        self.assertEqual(result[0]['X'], 10.1234 * 2)
        self.assertEqual(result[0]['Y'], 20.5678 * 2)
        self.assertEqual(result[1]['X'], 15.2222 * 2)
        self.assertEqual(result[1]['Y'], 25.9999 * 2)

    def test_add_offset_to_g_code(self):
        """
        Test addition of offset values to specified keys in the G-code.

        Ensures positional values are shifted correctly by the given
        offset dictionary.
        """
        offset = {'X': 5, 'Y': -5}
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.add_offset_to_g_code(g_code, offset)
        self.assertEqual(result[0]['X'], 10.1234 + 5)
        self.assertEqual(result[0]['Y'], 20.5678 - 5)
        self.assertEqual(result[1]['X'], 15.2222 + 5)
        self.assertEqual(result[1]['Y'], 25.9999 - 5)

    def test_convert_to_radians(self):
        """
        Test conversion of angular values from degrees to radians.

        Applies to keys A, B, C and all joint keys starting with 'RA'.
        """
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.convert_to_radians(g_code)
        self.assertAlmostEqual(result[0]['A'], np.radians(30.9876))
        self.assertAlmostEqual(result[1]['B'], np.radians(45.5432))
        self.assertAlmostEqual(result[2]['C'], np.radians(60.6543))
        self.assertAlmostEqual(result[3]['RA1'], np.radians(90.3333))

    def test_convert_to_degrees(self):
        """
        Test conversion of angular values from radians to degrees.

        Applies to keys A, B, C and all joint keys starting with 'RA'.
        """
        g_code = deepcopy(self.g_code)
        g_code[0]['A'] = np.radians(30.9876)
        g_code[1]['B'] = np.radians(45.5432)
        g_code[2]['C'] = np.radians(60.6543)
        g_code[3]['RA1'] = np.radians(90.3333)
        result = GCodeSimplifier.convert_to_degrees(g_code)
        self.assertAlmostEqual(result[0]['A'], 30.9876)
        self.assertAlmostEqual(result[1]['B'], 45.5432)
        self.assertAlmostEqual(result[2]['C'], 60.6543)
        self.assertAlmostEqual(result[3]['RA1'], 90.3333)

    def test_apply_feedrate(self):
        """
        Test application of feedrate to G-code lines with 'G' == 1.

        Ensures that 'F' key is added only to valid motion commands.
        """
        feedrate = 1000
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.apply_feedrate(g_code, feedrate)
        self.assertEqual(result[0]['F'], feedrate)
        self.assertNotIn('F', result[1])
        self.assertEqual(result[3]['F'], feedrate)

    def test_skip_command(self):
        """
        Test removal of specified keys from G-code commands.

        Verifies that only matching keys are removed and structure
        of remaining commands is preserved.
        """
        skip = {'X': 10.1234, 'RA1': 90.3333}
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.skip_command(g_code, skip)
        self.assertNotIn('X', result[0])
        self.assertIn('Y', result[0])
        self.assertNotIn('RA1', result[3])
        self.assertIn('G', result[3])


if __name__ == '__main__':
    unittest.main()
