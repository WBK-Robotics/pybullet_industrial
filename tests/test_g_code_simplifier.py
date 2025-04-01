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
            {'X': 10, 'Y': 20, 'A': 30, 'G': 1},
            {'X': 15, 'Y': 25, 'B': 45},
            {'Z': 5,  'C': 60},
            {'RA1': 90, 'G': 1}
        ]

    def test_round_pose_values(self):
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.round_pose_values(g_code, 2, 2)
        self.assertEqual(result[0]['X'], round(10, 2))
        self.assertEqual(result[0]['A'], round(30, 2))
        self.assertEqual(result[1]['X'], round(15, 2))
        self.assertEqual(result[2]['C'], round(60, 2))

    def test_round_joint_positions(self):
        g_code = deepcopy(self.g_code)
        # Modify joint values for testing
        g_code.append({'RA2': 45.67891, 'RAx': 50.12345})
        result = GCodeSimplifier.round_joint_positions(g_code, 3)
        self.assertEqual(result[3]['RA1'], round(90, 3))
        self.assertEqual(result[4]['RA2'], round(45.67891, 3))
        # Key 'RAx' should not be rounded since 'x' is not a digit
        self.assertEqual(result[4]['RAx'], 50.12345)

    def test_scale_g_code(self):
        scaling = 2.0
        keys = ['X', 'Y']
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.scale_g_code(g_code, scaling, keys)
        self.assertEqual(result[0]['X'], 20)
        self.assertEqual(result[0]['Y'], 40)
        self.assertEqual(result[1]['X'], 30)
        self.assertEqual(result[1]['Y'], 50)

    def test_add_offset_to_g_code(self):
        offset = {'X': 5, 'Y': -5}
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.add_offset_to_g_code(g_code, offset)
        self.assertEqual(result[0]['X'], 15)
        self.assertEqual(result[0]['Y'], 15)
        self.assertEqual(result[1]['X'], 20)
        self.assertEqual(result[1]['Y'], 20)

    def test_convert_to_radians(self):
        g_code = deepcopy(self.g_code)
        # Values in A, B, C and keys starting with 'RA'
        result = GCodeSimplifier.convert_to_radians(g_code)
        self.assertAlmostEqual(result[0]['A'],
                               np.radians(30))
        self.assertAlmostEqual(result[1]['B'],
                               np.radians(45))
        self.assertAlmostEqual(result[2]['C'],
                               np.radians(60))
        self.assertAlmostEqual(result[3]['RA1'],
                               np.radians(90))

    def test_convert_to_degrees(self):
        g_code = deepcopy(self.g_code)
        # Pre-convert angles to radians
        g_code[0]['A'] = np.radians(30)
        g_code[1]['B'] = np.radians(45)
        g_code[2]['C'] = np.radians(60)
        g_code[3]['RA1'] = np.radians(90)
        result = GCodeSimplifier.convert_to_degrees(g_code)
        self.assertAlmostEqual(result[0]['A'], 30)
        self.assertAlmostEqual(result[1]['B'], 45)
        self.assertAlmostEqual(result[2]['C'], 60)
        self.assertAlmostEqual(result[3]['RA1'], 90)

    def test_apply_feedrate(self):
        feedrate = 1000
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.apply_feedrate(g_code, feedrate)
        self.assertEqual(result[0]['F'], feedrate)
        # Second command does not have 'G' equal to 1
        self.assertNotIn('F', result[1])
        # Fourth command should have feedrate applied
        self.assertEqual(result[3]['F'], feedrate)

    def test_skip_command(self):
        skip = {'X': 10, 'RA1': 90}
        g_code = deepcopy(self.g_code)
        result = GCodeSimplifier.skip_command(g_code, skip)
        # In first command, X key should be removed
        self.assertNotIn('X', result[0])
        # Other keys remain intact
        self.assertIn('Y', result[0])
        # In fourth command, RA1 should be removed and command should remain
        self.assertNotIn('RA1', result[3])
        self.assertIn('G', result[3])


if __name__ == '__main__':
    unittest.main()
