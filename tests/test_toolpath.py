import unittest
import numpy as np
from pybullet_industrial.toolpath import ToolPath
import pybullet as p


class TestToolPath(unittest.TestCase):

    def setUp(self):
        self.positions = np.array([
            [0.0, 1.0, 2.0],
            [0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0]
        ])
        self.orientations = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0]
        ])
        self.tool_activations = np.array([True, False, True])
        self.path = ToolPath(self.positions.copy(), self.orientations.copy(),
                             self.tool_activations.copy())

    def test_init_defaults(self):
        path = ToolPath(self.positions.copy())
        self.assertEqual(path.orientations.shape, (4, 3))
        self.assertTrue(np.all(path.orientations[3] == 1))
        self.assertTrue(np.all(path.tool_activations == 0))

    def test_init_invalid_orientation_length(self):
        bad_orient = self.orientations[:, :2]
        with self.assertRaises(ValueError):
            ToolPath(self.positions, bad_orient, self.tool_activations)

    def test_init_invalid_activation_length(self):
        bad_activation = np.array([True, False])
        with self.assertRaises(ValueError):
            ToolPath(self.positions, self.orientations, bad_activation)

    def test_translate(self):
        vector = np.array([1.0, -1.0, 0.5])
        self.path.translate(vector)
        np.testing.assert_array_equal(
            self.path.positions,
            self.positions + vector[:, np.newaxis]
        )

    def test_get_start_pose(self):
        pos, ori = self.path.get_start_pose()
        np.testing.assert_array_equal(pos, self.positions[:, 0])
        np.testing.assert_array_equal(ori, self.orientations[:, 0])

    def test_rotate(self):
        quat = p.getQuaternionFromEuler([0, 0, np.pi / 2])
        original_positions = self.path.positions.copy()
        self.path.rotate(quat)
        self.assertEqual(self.path.positions.shape, original_positions.shape)
        self.assertEqual(self.path.orientations.shape, self.orientations.shape)

    def test_append(self):
        path2 = ToolPath(self.positions.copy(), self.orientations.copy(),
                         self.tool_activations.copy())
        self.path.append(path2)
        self.assertEqual(len(self.path), 6)
        np.testing.assert_array_equal(
            self.path.tool_activations,
            np.concatenate([self.tool_activations, self.tool_activations])
        )

    def test_prepend(self):
        path2 = ToolPath(self.positions.copy(), self.orientations.copy(),
                         self.tool_activations.copy())
        self.path.prepend(path2)
        self.assertEqual(len(self.path), 6)
        np.testing.assert_array_equal(
            self.path.tool_activations,
            np.concatenate([self.tool_activations, self.tool_activations])
        )

    def test_iteration(self):
        poses = list(self.path)
        self.assertEqual(len(poses), len(self.positions[0]))
        for i, (pos, ori, active) in enumerate(poses):
            np.testing.assert_array_equal(pos, self.positions[:, i])
            np.testing.assert_array_equal(ori, self.orientations[:, i])
            self.assertEqual(active, self.tool_activations[i])


if __name__ == '__main__':
    unittest.main()