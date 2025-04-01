import pybullet as p
from pybullet_industrial import RobotBase
from pybullet_industrial import linear_interpolation
from pybullet_industrial import circular_interpolation
from pybullet_industrial import ToolPath
from pybullet_industrial import JointPath
import numpy as np
import re

JOINT_KEY = ('RA')


class GCodeProcessor:
    """Initializes a GCodeProcessor object with the provided parameters.

    Args:
        g_code_input (str, optional): Simulated input G-code
        robot (RobotBase, optional): The robot which is controlled
        endeffector_list (list, optional): List of endeffectors to use
        m_commands (list, optional): M-commands to execute
        t_commands (list, optional): T-commands to execute
        offset (np.array, optional): Point which defines the origin
        axis (int, optional): The axis around which the circle
                                is interpolated .Defaults to 2 which
                                corresponds to the z-axis (0=x,1=y)
        interpolation_precision (int, optional): Precision of interpolation
        interpolation_approach (int, optional): Number of interpolations
                                                before determining precision
    """

    def __init__(self, g_code_input: str = None, robot: RobotBase = None,
                 endeffector_list: list = None,
                 m_commands: dict = None,
                 t_commands: dict = None,
                 offset: np.array = np.array([[0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0]]),
                 axis: int = 2, interpolation_precision: int = 0.01,
                 interpolation_approach: int = 1000):

        #  Converting G-Code into special list format
        if g_code_input is not None:
            self.g_code = self.read_g_code(g_code_input)

        # Initializing class variables
        self.new_point = []
        self.new_or = []
        self.last_point = []
        self.last_or = []
        self.r_val = 0
        self.robot = robot
        self.endeffector_list = endeffector_list
        self.active_endeffector = self.__get_active_endeffector()
        self.offset = offset
        self.axis = axis
        self.interpolation_precision = interpolation_precision
        self.interpolation_approach = interpolation_approach
        self.m_commands = m_commands
        self.t_commands = t_commands

        # Setting the default G-commands
        self.g_commands = {
            "54": [lambda: self.__g_54()],
            "500": [lambda: self.__g_500()],
            "17": [lambda: self.__g_17()],
            "18": [lambda: self.__g_18()],
            "19": [lambda: self.__g_19()]
        }

        if robot is not None:
            self.__calibrate_tool()
            self.joint_order = robot.get_moveable_joints()[0]

    @staticmethod
    def read_g_code(g_code_input: str):
        """Reads G-code row by row and saves the processed data in
        a list. Comments that start with % are ignored and all
        the other data is stored as it gets read in.

        Args:
            g_code_input (str): Source of G-code as a string

        Returns:
            list: Processed G-code as a list of dictionaries
        """

        g_code_input = g_code_input.splitlines()
        g_code = []

        for line in g_code_input:
            if not line.strip() or line.strip().startswith('%'):
                continue

            new_line = {}
            components = line.split()

            for component in components:
                if "=" in component:
                    key, value = component.split("=")
                    val = float(value) if '.' in value else int(value)
                else:
                    match = re.match(r'([A-Z]+)(-?\d*\.?\d*)',
                                     component, re.IGNORECASE)
                    if match:
                        key = match.group(1)
                        value = match.group(2)
                        val = float(value) if value and '.' in value else int(
                            value)

                # Convert key to uppercase
                key = key.upper()

                new_line[key] = val

            g_code.append(new_line)

        return g_code

    @staticmethod
    def tool_path_to_g_code(tool_path: ToolPath):
        """Converts a ToolPath object into a G-code representation.

        Each path point is converted by mapping its position and orientation.
        The orientation is transformed from a quaternion to Euler angles.

        Args:
            tool_path (ToolPath): ToolPath object to be converted.

        Returns:
            list: A list of dictionaries representing the G-code.
        """
        g_code = []
        for position, orientation, _ in tool_path:
            euler = p.getEulerFromQuaternion(orientation)
            g_line = {
                'G': 1,
                'X': position[0],
                'Y': position[1],
                'Z': position[2],
                'A': euler[0],
                'B': euler[1],
                'C': euler[2]
            }
            g_code.append(g_line)
        return g_code

    @staticmethod
    def joint_path_to_g_code(joint_path: JointPath):
        """Converts a JointPath object into a G-code representation.

        Joint positions are mapped to G-code commands. Each joint value is
        associated with a key derived from a predefined
        joint key and its index.

        Args:
            joint_path (JointPath): JointPath object to be converted.

        Returns:
            list: A list of dictionaries representing the G-code.
        """
        g_code = []
        for joint_positions, _ in joint_path:
            g_code_line = {}
            g_code_line['G'] = 1
            for i, joint_name in enumerate(joint_path.joint_order):
                g_code_line[JOINT_KEY + str(i + 1)] = joint_positions[
                    joint_name]
            g_code.append(g_code_line)
        return g_code

    def __iter__(self):
        """ Initialization of the the class variables which are responsible
        for the iteration

        Returns:
        self(GCodeProcessor): Iterator
        """
        self.__calibrate_tool()
        self.elementary_operations = []
        self.index_operation = 0
        self.index_g_code = 0
        self.path = []
        return self

    def __next__(self):
        """Switches between running elementary operation and reading
        in commands from the G-code to create elementary operations.
        Every line of the G-code causes the built
        of new elementary operations"""

        # Runs elementary operations
        if self.index_operation < len(self.elementary_operations):
            i = self.index_operation
            self.index_operation += 1
            self.elementary_operations[i]()

        # Reads the G-Code command to create elementary operations
        elif self.index_g_code < len(self.g_code):
            self.elementary_operations = []
            self.index_operation = 0

            i = self.index_g_code
            g_code_line = self.g_code[i]

            self.last_point = np.array(self.new_point)
            self.last_or = np.array(self.new_or)

            self.__create_elementary_operations(g_code_line)

            self.index_g_code += 1

            return self.g_code[i]

        else:
            raise StopIteration

    def __create_elementary_operations(self, g_code_line: dict):
        """Appends all the elemenatry operations which are necessary to execute
        the recent command. All the elementary operations are safed with the
        help of lambda calls.

        Args:
            cmd_type(str): Current G-command type
            cmd_int(int): Current G-command integer
        """

        interpolation_keys = {'X', 'Y', 'Z', 'A', 'B', 'C', 'R'}

        if 'G' in g_code_line:
            if any(key in g_code_line for key in interpolation_keys):
                path = self.__build_path()
                self.elementary_operations = \
                    self.__create_movement_operations(path)
            elif any(key.startswith(JOINT_KEY) for key in g_code_line):
                self.elementary_operations = \
                    self.__create_joint_movement_operations(g_code_line)
            elif g_code_line.get('G') > 3:
                for operation in self.g_commands[str(g_code_line.get('G'))]:
                    self.elementary_operations.append(lambda: operation())

        elif 'M' in g_code_line:
            for operation in self.m_commands[str(g_code_line.get('M'))]:
                self.elementary_operations.append(lambda: operation())

        elif 'T' in g_code_line:
            for operation in self.t_commands[str(g_code_line.get('T'))]:
                self.elementary_operations.append(lambda: operation())

            self.elementary_operations.append(lambda: self.__calibrate_tool())

    def __build_path(self):
        """Calculates a new point and new orientation based on the new
        coordinates and offset. Depending on the G-command type, a
        specific tool path is returned. G0 commands create a path
        with 2 interpolation steps, while higher G-1-2-3
        commands generate a path with the chosen precision.

        Returns:
            path(ToolPath): Interpolated tool path
        """

        g_code_line = self.g_code[self.index_g_code]
        # g_com = cmd[0][1]

        self.__build_new_point(g_code_line)

        if g_code_line['G'] == 0:
            path = self.__build_simple_path()
            orientation = p.getQuaternionFromEuler(self.new_or)
            path.orientations = np.transpose([orientation]
                                             * len(path.orientations[0]))
        else:
            path = self.__build_precise_path(g_code_line['G'])

        return path

    def __build_new_point(self, g_code_line: dict):
        """Calculates the new point of a G-Code command with respect
        to the current offset.

        Args:
            cmd(list): Current command line
        """

        variables = {'G': np.nan, 'X': np.nan, 'Y': np.nan, 'Z': np.nan,
                     'A': np.nan, 'B': np.nan, 'C': np.nan, 'R': np.nan,
                     'F': np.nan}

        # for val in cmd:
        for key, value in g_code_line.items():
            if key in variables:
                variables[key] = value
            else:
                raise KeyError("Variable '{}' is not defined.".format(key))

        xyz_val = np.array([variables['X'], variables['Y'],
                            variables['Z']])
        abc_val = np.array([variables['A'], variables['B'],
                            variables['C']])
        self.r_val = variables['R']

        # Setting the new point considering the offset
        self.new_point = np.array([0.0, 0.0, 0.0])
        for i, value in enumerate(xyz_val):
            if np.isnan(value):
                self.new_point[i] = self.last_point[i]
            else:
                self.new_point[i] = value + self.offset[0][i]

        # Setting the new orientation
        orientation = np.array([0.0, 0.0, 0.0])
        for i, value in enumerate(abc_val):
            if np.isnan(value):
                orientation[i] = self.last_or[i] - self.offset[1][i]
            else:
                orientation[i] = value

        orientation = p.getQuaternionFromEuler(orientation)
        orientation_offset = p.getQuaternionFromEuler(self.offset[1])

        # Transforming the oriention considering the offset
        point = np.array([0.0, 0.0, 0.0])
        _, self.new_or = p.multiplyTransforms(
            point, orientation, point, orientation_offset)

        self.new_or = p.getEulerFromQuaternion(self.new_or)

    def __build_simple_path(self):
        """ Returns the simple path of a G0-interpolation

        Returns:
            path(ToolPath): G0-toolpath
        """

        path = linear_interpolation(self.last_point,
                                    self.new_point, 2)

        return path

    def __build_precise_path(self, g_com: int):
        """Returns the percise path for G-2-3-Interpolations by calculating
        the neccessary amount of interpolation steps considering the precision
        of the interpolation.

        Args:
            g_com(int): Current G-command type
        """

        interpolation_steps = self.interpolation_approach
        percise_path = True
        start_or = p.getQuaternionFromEuler(self.last_or)
        end_or = p.getQuaternionFromEuler(self.new_or)

        for _ in range(2):

            # Building the Path if there is a linear interpolation
            if g_com == 1:
                path = linear_interpolation(self.last_point,
                                            self.new_point,
                                            interpolation_steps, start_or,
                                            end_or)

            # Building the path if there is a circular interpolation
            elif g_com in [2, 3]:
                if g_com == 2:
                    path = circular_interpolation(self.last_point,
                                                  self.new_point, self.r_val,
                                                  interpolation_steps,
                                                  self.axis, True,
                                                  start_or,
                                                  end_or)
                else:
                    path = circular_interpolation(self.last_point,
                                                  self.new_point, self.r_val,
                                                  interpolation_steps,
                                                  self.axis, False,
                                                  start_or,
                                                  end_or)
            # Calculating the total ditance
            if percise_path:
                percise_path = False
                total_distance = 0
                point_distance = 0
                previous_postion = self.last_point

                for position, _, _ in path:

                    # Calculating the point distance
                    point_distance = np.linalg.norm(
                        position - previous_postion)

                    # Adding point distnace to global distance
                    total_distance += point_distance
                    previous_postion = position

                interpolation_steps = total_distance/self.interpolation_precision
                interpolation_steps = int(np.ceil(interpolation_steps)) + 1

        return path

    def __create_movement_operations(self, path: ToolPath):
        """Returns a list of elementary operations to move the robot based
        on a given tool path and active endeffector.

        Args:
            path(ToolPath): input tool path

        Returns:
            elementary_operations(list): elementary operations to move robot
        """

        active = self.active_endeffector  # abbreviation
        elementary_operations = []

        for position, orientation, _ in path:
            if self.active_endeffector == -1:
                elementary_operations.append(
                    lambda i=position, j=orientation:
                    self.robot.set_endeffector_pose(i, j))

            else:
                elementary_operations.append(
                    lambda i=position, j=orientation:
                    self.endeffector_list[active].set_tool_pose(i, j))

        return elementary_operations

    def __get_active_endeffector(self):
        """Returns the index of the active endeffector.

        Returns:
            active_endeffector(int): index of the active endeffector
        """

        active_endeffector = -1

        if self.endeffector_list is not None:
            for n, i in enumerate(self.endeffector_list):
                if i.is_coupled():
                    active_endeffector = n
                    break
        return active_endeffector

    def __calibrate_tool(self):
        """This method sets the current postion of the active tool. This
        ensures a smooth transition between tool changes.
        """

        self.active_endeffector = self.__get_active_endeffector()
        actv = self.active_endeffector  # abbreviation

        if self.active_endeffector == -1:
            self.new_point = self.robot.get_endeffector_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                self.robot.get_endeffector_pose()[1])
            self.new_or = np.array(or_euler)
        else:
            self.new_point = self.endeffector_list[actv].get_tool_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                self.endeffector_list[actv].get_tool_pose()[1])
            self.new_or = np.array(or_euler)

    def __create_joint_movement_operations(self, g_code_line: dict):
        """Returns a list with a lambda function to set the joint position.

        Args:
            g_code_line(dict): Current G-code line

        Returns:
            elementary_operations(list)
        """
        joint_positions = {}
        for key, value in g_code_line.items():
            if key.startswith(JOINT_KEY):
                joint_name = self.joint_order[int(key[len(JOINT_KEY):])-1]
                joint_positions[joint_name] = value

        elementary_operations = [
            lambda: self.robot.set_joint_position(joint_positions)]

        return elementary_operations

    def __g_54(self):
        # Activation of the zero offset
        self.offset = np.array([self.last_point, self.last_or])

    def __g_500(self):
        # Deactivation of the zero offset
        self.offset = np.array([[0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])

    def __g_17(self):
        # Axis selelection for circular interpolation
        self.axis = 2  # X-Y Axis

    def __g_18(self):
        # Axis selelection for circular interpolation
        self.axis = 1  # X-Z Axis

    def __g_19(self):
        # Axis selelection for circular interpolation
        self.axis = 0  # Y-Z Axis
