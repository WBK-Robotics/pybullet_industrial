import unittest
import numpy as np
from pybullet_industrial.joint_path import JointPath  # Pfad ggf. anpassen


class TestJointPath(unittest.TestCase):

    def setUp(self):
        self.joint_order = ['q1', 'q2', 'q3']
        self.joint_values = np.array([
            [0.0, 0.1, 0.2],
            [0.1, 0.2, 0.3],
            [0.2, 0.3, 0.4]
        ])
        self.tool_activations = np.array([True, False, True])
        self.path = JointPath(self.joint_values, self.joint_order,
                              self.tool_activations)

    def test_init_valid(self):
        self.assertEqual(self.path.joint_order, self.joint_order)
        self.assertEqual(len(self.path), 3)
        np.testing.assert_array_equal(self.path.tool_activations,
                                      self.tool_activations)

    def test_init_invalid_shape(self):
        bad_values = self.joint_values[:2]
        with self.assertRaises(ValueError):
            JointPath(bad_values, self.joint_order)

    def test_init_invalid_tool_activations(self):
        bad_tool = np.array([True, False])  # too short
        with self.assertRaises(ValueError):
            JointPath(self.joint_values, self.joint_order, bad_tool)

    def test_get_joint_configuration(self):
        config = self.path.get_joint_configuration(1)
        expected = {'q1': 0.1, 'q2': 0.2, 'q3': 0.3}
        self.assertEqual(config, expected)

    def test_offset(self):
        offsets = {'q1': 0.5, 'q3': -0.1}
        self.path.offset(offsets)
        np.testing.assert_array_almost_equal(
            self.path.joint_values[0], [0.5, 0.6, 0.7])
        np.testing.assert_array_almost_equal(
            self.path.joint_values[2], [0.1, 0.2, 0.3])

    def test_append(self):
        other = JointPath(self.joint_values, self.joint_order,
                          self.tool_activations)
        self.path.append(other)
        self.assertEqual(len(self.path), 6)
        np.testing.assert_array_equal(
            self.path.tool_activations,
            np.concatenate([self.tool_activations, self.tool_activations])
        )

    def test_prepend(self):
        other = JointPath(self.joint_values, self.joint_order,
                          self.tool_activations)
        self.path.prepend(other)
        self.assertEqual(len(self.path), 6)
        np.testing.assert_array_equal(
            self.path.tool_activations,
            np.concatenate([self.tool_activations, self.tool_activations])
        )

    def test_append_mismatched_order(self):
        wrong_order = ['q3', 'q2', 'q1']
        other = JointPath(self.joint_values, wrong_order,
                          self.tool_activations)
        with self.assertRaises(ValueError):
            self.path.append(other)

    def test_iterator(self):
        expected_configs = [
            {'q1': 0.0, 'q2': 0.1, 'q3': 0.2},
            {'q1': 0.1, 'q2': 0.2, 'q3': 0.3},
            {'q1': 0.2, 'q2': 0.3, 'q3': 0.4},
        ]
        expected_tools = [True, False, True]

        for (config, tool), exp_conf, exp_tool in zip(self.path, expected_configs, expected_tools):
            self.assertEqual(config, exp_conf)
            self.assertEqual(tool, exp_tool)


if __name__ == '__main__':
    unittest.main()
