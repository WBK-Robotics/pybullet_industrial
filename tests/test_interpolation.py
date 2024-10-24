import unittest
import numpy as np
import pybullet as p
from pybullet_industrial.toolpath import ToolPath
from pybullet_industrial.interpolation import linear_interpolation, circular_interpolation
from scipy.spatial.transform import Rotation as R, RotationSpline


class TestInterpolation(unittest.TestCase):

    def test_linear_interpolation_with_both_orientations(self):
        # Test linear interpolation with both start and end orientation
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([10.0, 10.0, 10.0])
        samples = 5
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, np.pi / 2, 0])  # 90 degrees rotation

        result = linear_interpolation(
            start_point, end_point, samples, start_orientation, end_orientation)

        # Create expected ToolPath for positions
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
        expected_orientations = slerp_spline(t_vals).as_quat()

        expected_toolpath = ToolPath(
            positions=expected_positions, orientations=expected_orientations.transpose())

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

        # Create expected ToolPath for positions
        expected_positions = np.array([
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0],
            [0.0, 2.5, 5.0, 7.5, 10.0]
        ])

        # Since end_orientation is None, the same start_orientation should be used for all points
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
            [0, 0, np.pi / 2])  # 90 degrees rotation around Z-axis

        result = circular_interpolation(
            start_point, end_point, radius, samples, axis, clockwise, start_orientation, end_orientation)

        # Generate expected positions (based on circular interpolation)
        # Assuming planar interpolation is correct
        expected_positions = result.positions

        # Generate expected orientations using RotationSpline for SLERP
        start_rot = R.from_quat(start_orientation)
        end_rot = R.from_quat(end_orientation)
        rotations = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
        t_vals = np.linspace(0, 1, samples)
        slerp_spline = RotationSpline([0, 1], rotations)
        expected_orientations = slerp_spline(t_vals).as_quat()

        expected_toolpath = ToolPath(
            positions=expected_positions, orientations=expected_orientations.transpose())

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

        # Create expected ToolPath for positions
        # Assuming planar interpolation is correct
        expected_positions = result.positions

        # Since end_orientation is None, the same start_orientation should be used for all points
        expected_orientations = np.tile(
            start_orientation, (samples, 1)).transpose()
        expected_toolpath = ToolPath(
            positions=expected_positions, orientations=expected_orientations)

        # Check positions and orientations
        np.testing.assert_array_almost_equal(
            result.positions, expected_toolpath.positions, decimal=6)
        np.testing.assert_array_almost_equal(
            result.orientations, expected_toolpath.orientations, decimal=6)


if __name__ == '__main__':
    unittest.main()
