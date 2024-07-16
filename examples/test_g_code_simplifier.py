import unittest
import numpy as np
from g_code_simplifier import GCodeSimplifier


class TestGCodeSimplifier(unittest.TestCase):

    def setUp(self):
        # Sample input G-code data for testing
        self.simplfy_sample_g_code = [
            {'G': 1, 'X': 0.0, 'Y': 0.0, 'Z': 0.0,
                'A': -3.5, 'B': -3.5, 'C': -3.5},
            {'G': 1, 'X': 2.0, 'Y': 2.0, 'Z': 2.0,
                'A': -2.1, 'B': -2.1, 'C': -2.1},
            {'G': 1, 'X': 4.0, 'Y': 4.0, 'Z': 4.0,
                'A': -0.7, 'B': -0.7, 'C': -0.7},
            {'G': 1, 'X': 6.0, 'Y': 6.0, 'Z': 6.0, 'A': 0.7, 'B': 0.7, 'C': 0.7},
            {'G': 1, 'X': 8.0, 'Y': 8.0, 'Z': 8.0, 'A': 2.1, 'B': 2.1, 'C': 2.1},
            {'G': 1, 'X': 10.0, 'Y': 10.0, 'Z': 10.0, 'A': 3.5, 'B': 3.5, 'C': 3.5}
        ]

        self.transform_sample_g_code = [
            {'G': 1, 'X': 8.2459, 'Y': 1.3463, 'Z': 3.9874, 'A': -1.7321, 'B': -2.8943, 'C': 2.9876}]
        self.simplifier = GCodeSimplifier()

    def test_initialization(self):
        self.simplifier.g_code = self.simplfy_sample_g_code
        self.assertIsInstance(self.simplifier, GCodeSimplifier)
        self.assertEqual(self.simplifier.g_code, self.sample_g_code)
        self.assertIsNone(self.simplifier.input_points)
        self.assertIsNone(self.simplifier.input_orientations)

    def test_g_code_to_arrays(self):
        self.simplifier.g_code_to_arrays()
        self.assertIsNotNone(self.simplifier.input_points)
        self.assertIsNotNone(self.simplifier.input_orientations)
        self.assertEqual(len(self.simplifier.input_points), 6)
        self.assertEqual(len(self.simplifier.input_orientations), 6)

    def test_simplify_g_code(self):
        self.simplifier.simplify_g_code(epsilon=0.1)
        self.assertIsNotNone(self.simplifier.simplified_points)
        self.assertIsNotNone(self.simplifier.simplified_orientations)
        self.assertGreater(len(self.simplifier.input_points),
                           len(self.simplifier.simplified_points))
        self.assertGreater(len(self.simplifier.input_orientations), len(
            self.simplifier.simplified_orientations))

    def test_round_cartesian(self):
        self.simplifier.round_cartesian(round_xyz=2, round_abc=2)
        for g_code_line in self.simplifier.g_code:
            for key in ['X', 'Y', 'Z']:
                if key in g_code_line:
                    self.assertEqual(
                        len(str(g_code_line[key]).split('.')[-1]), 2)
            for key in ['A', 'B', 'C']:
                if key in g_code_line:
                    self.assertEqual(
                        len(str(g_code_line[key]).split('.')[-1]), 2)

    def test_round_joint_position(self):
        self.simplifier.round_joint_position(round_dec=2)
        search_keys = ['RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']
        for g_code_line in self.simplifier.g_code:
            for key in search_keys:
                if key in g_code_line:
                    self.assertEqual(
                        len(str(g_code_line[key]).split('.')[-1]), 2)

    def test_scale_g_code(self):
        scaling = 2.0
        keys_xyz = ['X', 'Y', 'Z']
        original_values = {key: [
            line[key] for line in self.simplifier.g_code if key in line] for key in keys_xyz}
        self.simplifier.scale_g_code(scaling, keys_xyz)
        for key in keys_xyz:
            for original, scaled in zip(original_values[key], [line[key] for line in self.simplifier.g_code if key in line]):
                self.assertEqual(scaled, original * scaling)

    def test_add_offset_to_g_code(self):
        offset_dict = {'X': 1.0, 'Y': 1.0, 'Z': 1.0}
        keys_xyz = ['X', 'Y', 'Z']
        original_values = {key: [
            line[key] for line in self.simplifier.g_code if key in line] for key in keys_xyz}
        self.simplifier.add_offset_to_g_code(offset_dict, keys_xyz)
        for key in keys_xyz:
            for original, offset in zip(original_values[key], [line[key] for line in self.simplifier.g_code if key in line]):
                self.assertEqual(offset, original - offset_dict[key])

    def test_convert_to_degrees(self):
        keys_abc = ['A', 'B', 'C']
        original_values = {key: [
            line[key] for line in self.simplifier.g_code if key in line] for key in keys_abc}
        self.simplifier.convert_to_degrees(
            round_dec=2, keys_abc=keys_abc, degrees=True)
        for key in keys_abc:
            for original, degree in zip(original_values[key], [line[key] for line in self.simplifier.g_code if key in line]):
                self.assertEqual(degree, round(math.degrees(original), 2))

    def test_apply_feedrate(self):
        feedrate = 1500
        self.simplifier.apply_feedrate(feedrate)
        for g_code_line in self.simplifier.g_code:
            if 'G' in g_code_line and g_code_line['G'] == 1:
                self.assertEqual(g_code_line.get('F'), feedrate)

    def test_skip_command(self):
        skip_keys = ['G2']
        self.simplifier.skip_command(skip_keys)
        for g_code_line in self.simplifier.g_code:
            for key in skip_keys:
                self.assertNotIn(key, g_code_line.values())


if __name__ == '__main__':
    unittest.main()
