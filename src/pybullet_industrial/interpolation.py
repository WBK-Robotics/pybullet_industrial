import numpy as np
import scipy.interpolate as sci
from scipy.spatial.transform import Rotation as R, RotationSpline

from pybullet_industrial.toolpath import ToolPath


def slerp(start_quat, end_quat, t_vals):
    """Performs Spherical Linear Interpolation (SLERP) between two quaternions

    Args:
        start_quat (np.array): The starting quaternion
        end_quat (np.array): The ending quaternion
        t_vals (np.array): Array of interpolation parameters (0 <= t <= 1)

    Returns:
        np.array: The array of interpolated quaternions
    """
    start_rot = R.from_quat(start_quat)
    end_rot = R.from_quat(end_quat)

    # Create the spline (SLERP) object from two quaternions
    slerp_rotations = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
    slerp_spline = RotationSpline([0, 1], slerp_rotations)

    # Perform SLERP interpolation at the time values
    interpolated_rots = slerp_spline(t_vals)

    return interpolated_rots.as_quat()


def build_circular_path(center: np.array, radius: float,
                        min_angle: float, max_angle: float, step_num: int, clockwise: bool = True):
    """Function which builds a circular path

    Args:
        center (np.array): the center of the circle
        radius (float): the radius of the circle
        min_angle (float): minimum angle of the circle path
        max_angle (float): maximum angle of the circle path
        step_num (int): the number of steps including min_angle and max_angle
        clockwise (bool): boolean value indicating if the interpolation is performed clockwise
                          or anticlockwise

    Returns:
        np.array: array of 2 dimensional path points
    """

    circular_path = np.zeros((2, step_num))
    for j in range(step_num):
        if clockwise:
            path_angle = min_angle - j * \
                (max_angle - min_angle) / (step_num - 1)
        else:
            path_angle = min_angle + j * \
                (max_angle - min_angle) / (step_num - 1)

        new_position = center + radius * \
            np.array([np.cos(path_angle), np.sin(path_angle)])
        circular_path[:, j] = new_position
    return circular_path


def linear_interpolation(start_point: np.array, end_point: np.array, samples: int,
                         start_orientation: np.array = None,
                         end_orientation: np.array = None):
    """Performs a linear interpolation between two points in 3D space

    Args:
        start_point (np.array): The start point of the interpolation
        end_point (np.array): The end point of the interpolation
        samples (int): The number of samples including start and end
        start_orientation (np.array): Start orientation as quaternion
        end_orientation (np.array): End orientation as quaternion

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """
    positions = np.linspace(start_point, end_point, num=samples)
    final_path = ToolPath(positions=positions.transpose())

    if start_orientation is not None:
        if end_orientation is None:
            end_orientation = start_orientation

        # Time values for SLERP interpolation
        t_vals = np.linspace(0, 1, samples)

        # Interpolate orientations using SLERP
        final_orientations = slerp(start_orientation, end_orientation, t_vals)

        final_path.orientations = final_orientations.transpose()

    return final_path


def planar_circular_interpolation(start_point: np.array, end_point: np.array,
                                  radius: float, samples: int, clockwise: bool = True):
    """Helper function which performs a circular interpolation in the x-y plane

    Args:
        start_point (np.array): The start point of the interpolation
        end_point (np.array): The end point of the interpolation
        radius (float): The radius of the circle
        samples (int): The number of including start and end
        clockwise (bool): boolean value indicating if the interpolation is performed clockwise
                            or anticlockwise

    Returns:
        np.array: The interpolated path
    """
    connecting_line = end_point-start_point
    distance_between_points = np.linalg.norm(connecting_line)
    if radius <= distance_between_points/2:
        raise ValueError("The radius needs to be at least " +
                         str(distance_between_points/2))

    center_distance_from_connecting_line = np.sqrt(
        radius**2-distance_between_points**2/4)

    if clockwise:
        orthogonal_vector = np.array(
            [connecting_line[1], -1*connecting_line[0]])
    else:
        orthogonal_vector = np.array(
            [-1*connecting_line[1], connecting_line[0]])

    circle_center = start_point+connecting_line/2+center_distance_from_connecting_line * \
        orthogonal_vector/np.linalg.norm(orthogonal_vector)

    angle_range = np.arccos(center_distance_from_connecting_line/radius)*2
    initial_angle = np.arctan2(
        start_point[1]-circle_center[1], start_point[0]-circle_center[0])

    planar_path = build_circular_path(
        circle_center, radius, initial_angle, initial_angle+angle_range, samples, clockwise)
    return planar_path


def circular_interpolation(start_point: np.array, end_point: np.array,
                           radius: float, samples: int, axis: int = 2,
                           clockwise: bool = True, start_orientation: np.array = None,
                           end_orientation: np.array = None):
    """Performs a circular interpolation between two points in 3D space

    Args:
        start_point (np.array): The start point of the interpolation
        end_point (np.array): The end point of the interpolation
        radius (float): The radius of the circle used for the interpolation
        samples (int): The number of samples including start and end
        axis (int, optional): The axis around which the circle is interpolated.
                              Defaults to 2 which corresponds to the z-axis (0=x, 1=y).
        clockwise (bool, optional): The direction of circular travel. Defaults to True.
        start_orientation (np.array): Start orientation as quaternion
        end_orientation (np.array): End orientation as quaternion

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """
    all_axis = [0, 1, 2]
    all_axis.remove(axis)

    # Interpolate positions in the x-y plane
    planar_start_point = np.array(
        [start_point[all_axis[0]], start_point[all_axis[1]]])
    planar_end_point = np.array(
        [end_point[all_axis[0]], end_point[all_axis[1]]])

    planar_path = planar_circular_interpolation(
        planar_start_point, planar_end_point, radius, samples, clockwise)

    positions = np.zeros((3, samples))
    for i in range(2):
        positions[all_axis[i]] = planar_path[i]
    positions[axis] = np.linspace(start_point[axis], end_point[axis], samples)

    final_path = ToolPath(positions=positions)

    if start_orientation is not None:
        if end_orientation is None:
            end_orientation = start_orientation

        # Time values for SLERP interpolation
        t_vals = np.linspace(0, 1, samples)

        # Interpolate orientations using SLERP
        final_orientations = slerp(start_orientation, end_orientation, t_vals)

        final_path.orientations = final_orientations.transpose()

    return final_path


def spline_interpolation(points: np.array, samples: int):
    """Interpolates between a number of points in cartesian space.

    Args:
        points (np.array(3,n)): A 3 dimensional array whith each dimension containing
                                   subsequent positions.
        samples (int): The number of samples used to interpolate

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """
    s = np.linspace(0, 1, len(points[0]))

    path = np.zeros((3, samples))
    cs_x = sci.CubicSpline(s, points[0])
    cs_y = sci.CubicSpline(s, points[1])
    cs_z = sci.CubicSpline(s, points[2])

    cs_s = np.linspace(0, 1, samples)
    path[0] = cs_x(cs_s)
    path[1] = cs_y(cs_s)
    path[2] = cs_z(cs_s)

    return ToolPath(path)
