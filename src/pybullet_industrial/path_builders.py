import numpy as np

from pybullet_industrial.interpolation import linear_interpolation, circular_interpolation


def build_box_path(center_position: np.array, dimensions: np.array,
                   radius: float, orientation: np.array, samples: int):
    """Build a box shaped path with rounded corners

    Args:
        center_position (np.array): The 3D center position of the box
        dimensions (np.array): the length and width of the box
        radius (float): the radius of the corners
        orientation (np.array): A quaternion describing the orientation of the box
        samples (int): The number of points in the box path

    Returns:
        ToolPath: The resulting Toolpath
    """
    corner_point_00 = np.array(
        [-0.5*dimensions[0], -0.5*dimensions[1]+radius, 0])
    corner_point_01 = np.array(
        [-0.5*dimensions[0], 0.5*dimensions[1]-radius, 0])
    corner_point_10 = np.array(
        [-0.5*dimensions[0]+radius, 0.5*dimensions[1], 0])
    corner_point_11 = np.array(
        [0.5*dimensions[0]-radius, 0.5*dimensions[1], 0])
    corner_point_20 = np.array(
        [0.5*dimensions[0], 0.5*dimensions[1]-radius, 0])
    corner_point_21 = np.array(
        [0.5*dimensions[0], -0.5*dimensions[1]+radius, 0])
    corner_point_30 = np.array(
        [0.5*dimensions[0]-radius, -0.5*dimensions[1], 0])
    corner_point_31 = np.array(
        [-0.5*dimensions[0]+radius, -0.5*dimensions[1], 0])

    linear_length = (np.linalg.norm(corner_point_00-corner_point_01) +
                     np.linalg.norm(corner_point_10-corner_point_11) +
                     np.linalg.norm(corner_point_20-corner_point_21) +
                     np.linalg.norm(corner_point_30-corner_point_31))

    circle_length = 2*np.pi*radius

    linear_samples = int(0.25*linear_length /
                         (linear_length+circle_length)*samples)
    circle_samples = int(0.25*circle_length /
                         (linear_length+circle_length)*samples)

    box_path = linear_interpolation(
        corner_point_00, corner_point_01, linear_samples)
    side_1 = linear_interpolation(
        corner_point_10, corner_point_11, linear_samples)
    side_2 = linear_interpolation(
        corner_point_20, corner_point_21, linear_samples)
    side_3 = linear_interpolation(
        corner_point_30, corner_point_31, linear_samples)

    corner_0 = circular_interpolation(
        corner_point_01, corner_point_10, radius, circle_samples)
    corner_1 = circular_interpolation(
        corner_point_11, corner_point_20, radius, circle_samples)
    corner_2 = circular_interpolation(
        corner_point_21, corner_point_30, radius, circle_samples)
    corner_3 = circular_interpolation(
        corner_point_31, corner_point_00, radius, circle_samples)

    box_path.append(corner_0)
    box_path.append(side_1)
    box_path.append(corner_1)
    box_path.append(side_2)
    box_path.append(corner_2)
    box_path.append(side_3)
    box_path.append(corner_3)

    box_path.rotate(orientation)
    box_path.translate(center_position)

    return box_path
