import unittest
from g_code_simplifier import GCodeSimplifier


class TestGCodeSimplifier(unittest.TestCase):

    def test_transform_g_code(self):
        g_code = [
            {'G': 1, 'X': 1.123, 'Y': 2.123, 'Z': 3.123,
                'A': -1.123, 'B': -0.123, 'C': 1.123, 'M': 12},
            {'G': 32},
            {'G': 0, 'RA1': -3.123, 'RA2': -2.123, 'RA3': -1.123,
                'RA4': 1.123, 'RA5': 2.123, 'RA6': 3.123}
        ]
        g_code = GCodeSimplifier.skip_command(g_code, {'G': 32, 'M': 12})
        g_code = GCodeSimplifier.convert_to_degrees(g_code)
        g_code = GCodeSimplifier.convert_to_radians(g_code)
        g_code = GCodeSimplifier.round_joint_positions(g_code, 1)
        g_code = GCodeSimplifier.round_cartesian(g_code, 1, 1)
        g_code = GCodeSimplifier.apply_feedrate(g_code, 5000)
        g_code = GCodeSimplifier.add_offset_to_g_code(
            g_code, {'X': -0.1, 'Y': -0.1, 'Z': -0.1})
        g_code = GCodeSimplifier.scale_g_code(g_code, 2, ['X', 'Y', 'Z'])

        desired_g_code = [{'G': 1, 'X': 2.0, 'Y': 4.0, 'Z': 6.0,
                           'A': -1.1, 'B': -0.1, 'C': 1.1, 'F': 5000},
                          {'G': 0, 'RA1': -3.1, 'RA2': -2.1, 'RA3': -1.1,
                           'RA4': 1.1, 'RA5': 2.1, 'RA6': 3.1}]
        self.assertEqual(g_code, desired_g_code)

    def test_cartesian_simplification(self):


if __name__ == '__main__':
    unittest.main()
