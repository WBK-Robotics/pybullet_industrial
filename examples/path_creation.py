import pybullet as p
import pybullet_industrial as pi
import numpy as np
import scipy.interpolate as sci


class ToolPath:

    def __init__(self, positions, orientations=None, tool_acivations=None):
        """A Base object for representing an manipulating toolpaths.

        Args:
            positions (numpy.array(3,n)): A 3 dimensional array whith each dimension containing
                                          subsequent positions.
            orientations (numpy.array(4,n), optional): A 4 dimensional array where each dimension
                                                       describes a subsequent quaternion.
                                                       Defaults to None in which case the orientation
                                                       of the world coordinate system is assumed.
            tool_acivations ([type], optional): A 1 dimensional array with boolean values describing
                                                wheter a tool is active at a given path pose.
                                                Defaults to None in which case the tool is always
                                                inactive.

        Raises:
            ValueError: If all given input arrays are different lengths.
        """
        self.positions = positions
        if orientations == None:
            self.orientations = np.zeros((4, len(self.positions[0])))
            self.orientations[3] = 1
        else:
            if len(orientations[0]) != len(positions[0]):
                raise ValueError(
                    "The position and orientation paths need to have the same length")
            self.orientations = orientations
        if tool_acivations == None:
            self.tool_activations = np.zeros(len(self.positions[0]))
        else:
            if len(tool_acivations[0]) != len(positions[0]):
                raise ValueError(
                    "The position and tool activation paths need to have the same length")
            self.tool_activations = tool_acivations

    def translate(self, vector):
        """Translates the whole path by a given vector

        Args:
            vector ([type]): A 3D vector describing the path translation
        """
        self.positions[0] += vector[0]
        self.positions[1] += vector[1]
        self.positions[2] += vector[2]

    def rotate(self, quaternion):
        """Rotates the vector by a given quaternion.
           Can be combined with pybullet.getQuaternionFromEuler() for easier usage.

        Args:
            quaternion ([type]): A 4 dimensional quaternion as a list or numpy array
        """
        path_positions = np.transpose(self.positions)
        path_orientations = np.transpose(self.orientations)

        rot_matrix = p.getMatrixFromQuaternion(quaternion)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        for i in range(len(self)):
            path_positions[i] = rot_matrix@path_positions[i]
            path_orientations[i] = pi.quaternion_multiply(
                path_orientations[i], quaternion)

        self.positions = np.transpose(path_positions)
        self.orientations = np.transpose(path_orientations)

    def draw(self, pose=False, color=[0, 0, 1]):
        """Function which draws the path into the Debugin GUI.
           The path can either be a line representing the positions or a series of coordinate systems
           representing the whole pose

        Args:
            orientation (bool, optional): Flag which determins if only the path position is shown
                                          or the full pose. Defaults to False.
            color (list, optional): The color of the line used for position only drawing.
                                    Defaults to [0, 0, 1].
        """
        if pose == False:
            pi.draw_path(self.positions, color)
        else:
            path_positions = np.transpose(self.positions)
            path_orientations = np.transpose(self.orientations)
            for i in range(len(self)):
                pi.draw_coordinate_system(
                    path_positions[i], path_orientations[i])

    def append(self, tool_path):
        """Appends a given ToolPath object to the end of this tool path.

        Args:
            tool_path (ToolPath): Another ToolPath object.
        """
        self.positions = np.append(self.positions, tool_path.positions, axis=1)
        self.orientations = np.append(
            self.orientations, tool_path.orientations, axis=1)
        self.tool_activations = np.append(
            self.tool_activations, tool_path.tool_activations)

    def prepend(self, tool_path):
        """Prepends a given ToolPath object to the start of this tool path.

        Args:
            tool_path (ToolPath): Another ToolPath object.
        """
        self.positions = np.append(tool_path.positions, self.positions, axis=1)
        self.orientations = np.append(
            tool_path.orientations, self.orientations, axis=1)
        self.tool_activations = np.append(
            tool_path.tool_activations, self.tool_activations)

    def __len__(self):
        return len(self.positions[0])

    def __iter__(self):
        self.current_index = 0
        return self

    def __next__(self):
        if self.current_index <= len(self)-1:
            i = self.current_index
            self.current_index += 1
            return self.positions[:, i], self.orientations[:, i], self.tool_activations[i]
        else:
            raise StopIteration


def build_circular_path(center, radius, min_angle, max_angle, step_num, clockwise=True):
    """Function which builds a circular path

    Args:
        center (array): the center of the circle
        radius (float): the radius of the circle
        min_angle (float): minimum angle of the circle path
        max_angle (float): maximum angle of the circle path
        steps (int): the number of steps between min_angle and max_angle

    Returns:
        array: array of 2 dimensional path points
    """

    circular_path = np.zeros((2, step_num))
    for j in range(step_num):
        if clockwise:
            path_angle = min_angle-j*(max_angle-min_angle)/step_num
        else:
            path_angle = min_angle+j*(max_angle-min_angle)/step_num
        new_position = center + radius * \
            np.array([np.cos(path_angle), np.sin(path_angle)])
        circular_path[:, j] = new_position
    return circular_path


def linear_interpolation(start_point, end_point, samples):
    """Performs a linear interpolation betwenn two points in 3D space

    Args:
        start_point (numpy.array): The start point of the interpolation
        end_point (numpy.array): The end point of the interpolation
        samples (int): The number of samples used to interpolate

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """
    final_path = np.linspace(start_point, end_point, num=samples)
    return ToolPath(final_path.transpose())


def planar_circular_interpolation(start_point, end_point, radius, samples, clockwise=True):
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


def circular_interpolation(start_point, end_point, radius, samples, axis=2, clockwise=True):
    """AI is creating summary for circular_interpolation

    Args:
        start_point (numpy.array): The start point of the interpolation
        end_point (numpy.array): The end point of the interpolation
        radius ([type]): The radius of the circle used for the interpolation
        samples (int): The number of samples used to interpolate
        axis (int, optional): The axis around which the circle is interpolated.
                              Defaults to 2 which corresponds to the z-axis (0=x,1=y).
        clockwise (bool, optional): The direction of circular travel. Defaults to True.

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """

    all_axis = [0, 1, 2]
    all_axis.remove(axis)
    planar_start_point = np.array(
        [start_point[all_axis[0]], start_point[all_axis[1]]])
    planar_end_point = np.array(
        [end_point[all_axis[0]], end_point[all_axis[1]]])

    planar_path = planar_circular_interpolation(
        planar_start_point, planar_end_point, radius, samples, clockwise)

    path = np.zeros((3, samples))
    for i in range(2):
        path[all_axis[i]] = planar_path[i]
    path[axis] = np.linspace(start_point[axis], end_point[axis], samples)
    return ToolPath(path)


def spline_interpolation(points, samples):
    """Interpolates between a number of points in cartesian space.

    Args:
        points (numpy.array(3,n)): A 3 dimensional array whith each dimension containing
                                   subsequent positions.
        samples (int): The number of samples used to interpolate

    Returns:
        ToolPath: A ToolPath object of the interpolated path
    """
    s = np.linspace(0, 1, len(points[0]))

    path = np.zeros((3, samples))
    print(points[0], s)
    cs_x = sci.CubicSpline(s, points[0])
    cs_y = sci.CubicSpline(s, points[1])
    cs_z = sci.CubicSpline(s, points[2])

    cs_s = np.linspace(0, 1, samples)
    path[0] = cs_x(cs_s)
    path[1] = cs_y(cs_s)
    path[2] = cs_z(cs_s)

    return ToolPath(path)


if __name__ == "__main__":

    p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # ++ quadrant
    org_test_path = linear_interpolation(
        [0, 0, 0], [1/np.sqrt(2), 1/np.sqrt(2), 0], 10)
    org_test_path.draw(color=[1, 0, 0])

    test_path = circular_interpolation(
        np.array([0, 1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[1, 0, 0])

    # +- quadrant
    test_path = linear_interpolation(
        [1, -1, 0], [1-1/np.sqrt(2), -1+1/np.sqrt(2), 0], 10)
    test_path.draw(color=[0, 1, 0])

    test_path = circular_interpolation(
        np.array([0, -1, 0]), np.array([1, 0, 0]), 1, 50)
    test_path.draw(color=[0, 1, 0])

    # -- quadrant
    test_path = circular_interpolation(
        np.array([0, -1, 0]), np.array([-1, 0, 0.5]), 1, 50)
    test_path.draw(color=[0, 0, 1])

    # -+ quadrant
    test_path = circular_interpolation(
        np.array([0, 1, 0.5]), np.array([-1, 0, 0.5]), 1, 50, clockwise=False)
    test_path.draw(color=[0, 1, 1])

    test_path = circular_interpolation(
        np.array([0, 0, 0]), np.array([0, 1, 1]), 1, 50, axis=0)
    test_path.draw(color=[1, 1, 0])

    test_path = circular_interpolation(
        np.array([0, 0, 0]), np.array([1, 0, 1]), 1, 50, axis=1)
    test_path.draw(color=[1, 1, 0])

    test_path = spline_interpolation(
        [[0, 1, 0], [0, 0, 1], [0, 1, 1]], samples=50)
    test_path.draw(color=[1, 0.5, 0.5])

    test_path.translate([0, 0, 1])
    test_path.draw(pose=True)

    test_path.append(org_test_path)
    test_path.prepend(org_test_path)

    test_path.rotate(p.getQuaternionFromEuler([np.pi/2, 0, 0]))
    test_path.draw(pose=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        p.stepSimulation()
        for position, orientation in test_path:
            print(position)
