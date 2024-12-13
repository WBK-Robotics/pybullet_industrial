import unittest
import numpy as np
import pybullet as p
from pybullet_industrial.toolpath import ToolPath
from pybullet_industrial.interpolation import (
    linear_interpolation,
    circular_interpolation,
)


class TestInterpolation(unittest.TestCase):

    def test_linear_interpolation(self):
        """
        Test linear interpolation function with different orientation cases.

        This test covers two scenarios:
        1. Interpolation with both start and end orientations.
        2. Interpolation with only the start orientation
           (end orientation is None).

        It verifies that the interpolated positions and orientations match the
        expected values.

        Test cases:
        - Case 1: Both start and end orientations are provided.
        - Case 2: Only the start orientation is provided
          (end orientation is None).

        Assertions:
        - Checks if the interpolated positions and orientations are almost
          equal to the expected values.
        """

        # Case 1: Test with both start and end orientations
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([10.0, 10.0, 10.0])
        samples = 5
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        end_orientation = p.getQuaternionFromEuler(
            [0, np.pi / 2, 0]
        )  # 90 degrees rotation

        result_with_both_orientations = linear_interpolation(
            start_point, end_point, samples, start_orientation, end_orientation
        )

        # Expected positions for both orientations case
        expected_positions = np.array(
            [
                [0.0, 2.5, 5.0, 7.5, 10.0],
                [0.0, 2.5, 5.0, 7.5, 10.0],
                [0.0, 2.5, 5.0, 7.5, 10.0],
            ]
        )

        # Expected orientations
        expected_orientations_both = np.array(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.19509032, 0.0, 0.98078528],
                [0.0, 0.38268343, 0.0, 0.92387953],
                [0.0, 0.55557023, 0.0, 0.83146961],
                [0.0, 0.70710678, 0.0, 0.70710678],
            ]
        )

        expected_toolpath_with_both = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_both.transpose(),
        )

        # Check positions and orientations for both orientations case
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.positions,
            expected_toolpath_with_both.positions,
            decimal=6,
        )
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.orientations,
            expected_toolpath_with_both.orientations,
            decimal=6,
        )

        # Case 2: Test with only start orientation (end_orientation is None)
        result_with_only_start_orientation = linear_interpolation(
            start_point, end_point, samples, start_orientation,
            end_orientation=None
        )

        # Expected orientations for only start orientation case
        expected_orientations_start_only = np.tile(
            start_orientation, (samples, 1)
        ).transpose()

        expected_toolpath_with_start_only = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_start_only
        )

        # Check positions and orientations for only start orientation case
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.positions,
            expected_toolpath_with_start_only.positions,
            decimal=6,
        )
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.orientations,
            expected_toolpath_with_start_only.orientations,
            decimal=6,
        ),

    def test_circular_interpolation(self):
        """
        Test the circular_interpolation function for generating circular paths
        with specified orientations.

        This test covers two cases:
        1. Circular interpolation with both start and
           end orientations specified.
        2. Circular interpolation with only the start orientation specified
           (end orientation is None).

        The test verifies that the generated positions and orientations match
        the expected values.

        Assertions:
            - Positions and orientations for both start and end orientations.
            - Positions and orientations for only the start orientation.
        """

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
            [0, 0, np.pi / 2]
        )  # 90 degrees rotation around Z-axis

        result_with_both_orientations = circular_interpolation(
            start_point,
            end_point,
            radius,
            samples,
            axis,
            clockwise,
            start_orientation,
            end_orientation,
        )

        # Expected positions for both orientations case
        expected_positions = np.array(
            [
                [1.0, 0.61731657, 0.29289322, 0.07612047, 0.0],
                [0.0, 0.07612047, 0.29289322, 0.61731657, 1.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )

        # Generate expected orientations
        expected_orientations_both = np.array(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 0.19509032, 0.98078528],
                [0.0, 0.0, 0.38268343, 0.92387953],
                [0.0, 0.0, 0.55557023, 0.83146961],
                [0.0, 0.0, 0.70710678, 0.70710678],
            ]
        )

        expected_toolpath_with_both = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_both.transpose(),
        )

        # Check positions and orientations for both orientations case
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.positions,
            expected_toolpath_with_both.positions,
            decimal=6,
        )
        np.testing.assert_array_almost_equal(
            result_with_both_orientations.orientations,
            expected_toolpath_with_both.orientations,
            decimal=6,
        )

        # Case 2: Test with only start orientation (end_orientation is None)
        result_with_only_start_orientation = circular_interpolation(
            start_point,
            end_point,
            radius,
            samples,
            axis,
            clockwise,
            start_orientation,
            end_orientation=None,
        )

        # Expected orientations for only start orientation case
        expected_orientations_start_only = np.tile(
            start_orientation, (samples, 1)
        ).transpose()

        expected_toolpath_with_start_only = ToolPath(
            positions=expected_positions,
            orientations=expected_orientations_start_only
        )

        # Check positions and orientations for only start orientation case
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.positions,
            expected_toolpath_with_start_only.positions,
            decimal=6,
        )
        np.testing.assert_array_almost_equal(
            result_with_only_start_orientation.orientations,
            expected_toolpath_with_start_only.orientations,
            decimal=6,
        )

    def test_zero_sample_size(self):
        """
        Test interpolation functions with only start and end point & zero
        samples in between.

        This test checks the linear and circular interpolation functions with
        a sample size of 2.It verifies that the generated paths match the
        expected positions and orientations.
        """
        start_point = np.array([1.0, 0.0, 0.0])
        end_point = np.array([0.0, 1.0, 0.0])
        radius = 1.0
        samples = 2
        axis = 2  # Z-axis
        clockwise = True
        start_orientation = p.getQuaternionFromEuler(
            [0, 0, 0])  # Neutral orientation
        end_orientation = p.getQuaternionFromEuler(
            [0, 0, np.pi / 2]
        )  # 90 degrees rotation around Z-axis

        linear_path = linear_interpolation(
            start_point, end_point, samples, start_orientation, end_orientation
        )
        circular_path = circular_interpolation(
            start_point,
            end_point,
            radius,
            samples,
            axis,
            clockwise,
            start_orientation,
            end_orientation,
        )

        expected_position = np.array([[1, 0], [0, 1], [0, 0]])
        expected_orientation = np.array(
            [[0, 0], [0, 0], [0, 0.70710678], [1, 0.70710678]]
        )

        expected_toolpath = ToolPath(expected_position, expected_orientation)

        np.testing.assert_array_almost_equal(
            linear_path.positions, expected_toolpath.positions, decimal=6
        )
        np.testing.assert_array_almost_equal(
            circular_path.positions, expected_toolpath.positions, decimal=6
        )
        np.testing.assert_array_almost_equal(
            linear_path.orientations, expected_toolpath.orientations, decimal=6
        )
        np.testing.assert_array_almost_equal(
            linear_path.orientations, expected_toolpath.orientations, decimal=6
        )


if __name__ == "__main__":
    unittest.main()
