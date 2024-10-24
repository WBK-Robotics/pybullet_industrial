import unittest
import numpy as np
import pybullet as p
from pybullet_industrial.toolpath import ToolPath
from pybullet_industrial.interpolation import linear_interpolation, circular_interpolation


class TestInterpolation(unittest.TestCase):

    def test_linear_interpolation_with_both_orientations(self):
        # Test linear interpolation with both start and end orientation
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([10.0, 10.0, 10.0])
        samples = 5
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, np.pi/2, 0])  # 90 degrees rotation

        result = linear_interpolation(
            start_point, end_point, samples, start_orientation, end_orientation)

        # Create expected ToolPath for positions and orientations
        expected_positions = np.array([
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0]
        ])
        expected_orientations = np.array([p.getQuaternionFromEuler(
            [0, angle, 0]) for angle in np.linspace(0, np.pi/2, samples)]).transpose()
        expected_toolpath = ToolPath(
            positions=expected_positions, orientations=expected_orientations)

        # Check positions and orientations
        np.testing.assert_array_almost_equal(
            result.positions, expected_toolpath.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result.orientations, expected_toolpath.orientations, decimal=6)

    def test_linear_interpolation_with_only_start_orientation(self):
        # Test linear interpolation with only start orientation (end_orientation is None)
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([10.0, 10.0, 10.0])
        samples = 5
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation

        result = linear_interpolation(
            start_point, end_point, samples, start_orientation, end_orientation=None)

        # Create expected ToolPath for positions and orientations (orientations should remain the same)
        expected_positions = np.array([
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0]
        ])
        expected_orientations = np.tile(
            start_orientation, (samples, 1)).transpose()
        expected_toolpath = ToolPath(
            positions=expected_positions, orientations=expected_orientations)

        # Check positions and orientations
        np.testing.assert_array_almost_equal(
            result.positions, expected_toolpath.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result.orientations, expected_toolpath.orientations, decimal=6)

    def test_circular_interpolation_with_both_orientations(self):
        # Test circular interpolation with both start and end orientation
        start_point = np.array([1.0, 0.0, 0.0])
        end_point = np.array([0.0, 1.0, 0.0])
        radius = 1.0
        samples = 5
        axis = 2  # Z-axis
        clockwise = True
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, 0, np.pi/2])  # 90 degrees rotation around Z-axis

        result = circular_interpolation(
            start_point, end_point, radius, samples, axis, clockwise, start_orientation, end_orientation)

        # Create expected ToolPath for positions and orientations
        expected_orientations = np.array([p.getQuaternionFromEuler(
            [0, 0, angle]) for angle in np.linspace(0, np.pi/2, samples)]).transpose()
        expected_toolpath = ToolPath(
            positions=result.positions, orientations=expected_orientations)

        # Check positions and orientations
        np.testing.assert_array_almost_equal(
            result.positions, expected_toolpath.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result.orientations, expected_toolpath.orientations, decimal=6)

    def test_circular_interpolation_with_only_start_orientation(self):
        # Test circular interpolation with only start orientation (end_orientation is None)
        start_point = np.array([1.0, 0.0, 0.0])
        end_point = np.array([0.0, 1.0, 0.0])
        radius = 1.0
        samples = 5
        axis = 2  # Z-axis
        clockwise = True
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation

        result = circular_interpolation(
            start_point, end_point, radius, samples, axis, clockwise, start_orientation, end_orientation=None)

        # Create expected ToolPath for positions and orientations (orientations should remain the same)
        expected_orientations = np.tile(
            start_orientation, (samples, 1)).transpose()
        expected_toolpath = ToolPath(
            positions=result.positions, orientations=expected_orientations)

        # Check positions and orientations
        np.testing.assert_array_almost_equal(
            result.positions, expected_toolpath.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result.orientations, expected_toolpath.orientations, decimal=6)


if __name__ == '__main__':
    unittest.main()
