import unittest
import numpy as np
import pybullet as p
from pybullet_industrial.toolpath import ToolPath
from pybullet_industrial.interpolation import linear_interpolation, circular_interpolation
from scipy.spatial.transform import Rotation as R, RotationSpline


class TestInterpolation(unittest.TestCase):

    def test_linear_interpolation(self):
        # Test both cases for linear interpolation

        # Case 1: Test with both start and end orientations
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([10.0, 10.0, 10.0])
        samples = 5
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, np.pi / 2, 0])  # 90 degrees rotation

        result_with_both_orientations = linear_interpolation(
            start_point, end_point, samples,
            start_orientation, end_orientation)

        # Expected positions for both orientations case
        expected_positions = np.array([
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0]
        ])

        # Generate expected orientations using RotationSpline for SLERP
        start_rot = R.from_quat(start_orientation)
        end_rot = R.from_quat(end_orientation)
        rotations = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
        t_vals = np.linspace(0, 1, samples)
        slerp_spline = RotationSpline([0, 1], rotations)
        expected_orientations_both = slerp_spline(t_vals).as_quat()

        expected_toolpath_with_both = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_both.transpose())

        # Check positions and orientations for both orientations case
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.positions,
            expected_toolpath_with_both.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.orientations,
            expected_toolpath_with_both.orientations, decimal=6)

        # Case 2: Test with only start orientation (end_orientation is None)
        result_with_only_start_orientation = linear_interpolation(
            start_point, end_point, samples,
            start_orientation, end_orientation=None)

        # Expected orientations for only start orientation case
        expected_orientations_start_only = np.tile(
            start_orientation, (samples, 1)).transpose()

        expected_toolpath_with_start_only = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_start_only)

        # Check positions and orientations for only start orientation case
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.positions,
            expected_toolpath_with_start_only.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.orientations,
            expected_toolpath_with_start_only.orientations, decimal=6)

    def test_circular_interpolation(self):
        # Test both cases for circular interpolation

        # Case 1: Test with both start and end orientations
        start_point = np.array([1.0, 0.0, 0.0])
        end_point = np.array([0.0, 1.0, 0.0])
        radius = 1.0
        samples = 5
        axis = 2  # Z-axis
        clockwise = True
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, 0, np.pi / 2])  # 90 degrees rotation around Z-axis

        result_with_both_orientations = circular_interpolation(
            start_point, end_point, radius, samples, axis, clockwise,
            start_orientation, end_orientation)

        # Expected positions for both orientations case (using the provided array)
        expected_positions = np.array([
            [1.0, 0.61731657, 0.29289322, 0.07612047, 0.0],
            [0.0, 0.07612047, 0.29289322, 0.61731657, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0]
        ])

        # Generate expected orientations using RotationSpline for SLERP
        start_rot = R.from_quat(start_orientation)
        end_rot = R.from_quat(end_orientation)
        rotations = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
        t_vals = np.linspace(0, 1, samples)
        slerp_spline = RotationSpline([0, 1], rotations)
        expected_orientations_both = slerp_spline(t_vals).as_quat()

        expected_toolpath_with_both = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_both.transpose())

        # Check positions and orientations for both orientations case
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.positions,
            expected_toolpath_with_both.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.orientations,
            expected_toolpath_with_both.orientations, decimal=6)

        # Case 2: Test with only start orientation (end_orientation is None)
        result_with_only_start_orientation = circular_interpolation(
            start_point, end_point, radius, samples, axis, clockwise,
            start_orientation, end_orientation=None)

        # Expected orientations for only start orientation case
        expected_orientations_start_only = np.tile(
            start_orientation, (samples, 1)).transpose()

        expected_toolpath_with_start_only = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_start_only)

        # Check positions and orientations for only start orientation case
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.positions,
            expected_toolpath_with_start_only.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.orientations,
            expected_toolpath_with_start_only.orientations, decimal=6)

    def test_sample_size(self):
        start_point = np.array([1.0, 0.0, 0.0])
        end_point = np.array([0.0, 1.0, 0.0])
        radius = 1.0
        samples = 5
        axis = 2  # Z-axis
        clockwise = True
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, 0, np.pi / 2])  # 90 degrees rotation around Z-axis


if __name__ == '__main__':
    unittest.main()
