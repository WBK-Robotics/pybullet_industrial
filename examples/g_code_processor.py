import pybullet as p
from pybullet_industrial import RobotBase
from pybullet_industrial import linear_interpolation
from pybullet_industrial import circular_interpolation
from pybullet_industrial import ToolPath
import numpy as np


class GCodeProcessor:

    def __init__(self, gcode=None, robot: RobotBase = None,
                 endeffector_list: list = None,
                 m_commands: list = None,
                 t_commands: list = None,
                 offset: np.array = np.array([[0.0, 0.0, 0.0],
                                              [0.0, 0.0, 0.0]]),
                 axis: int = 2, interpolation_steps: int = 10,
                 sleep: int = 0.0001):
        """Initialize a PathMover object with the provided parameters.

        Args:
            robot (pi.RobotBase, optional): The robot which is controlled.
            filename (str, optional): The name of a G-code file to read.
            endeffector_list (list, optional): List of end effectors to use.
            m_commands (list, optional): M-commands to execute.
            t_commands (list, optional): T-commands to execute.
            offset (np.array, optional): The offset for the path points.
            axis (int, optional): The axis around which the circle
                                  is interpolated.Defaults to 2 which
                                  corresponds to the z-axis (0=x,1=y)
            interpolation_steps (int, optional): Number of interpolation steps.

        Returns:
            None
        """
        # Implemented for the Iteration
        self.gcode = gcode

        self.new_point = []
        self.new_or = []
        self.last_point = []
        self.last_or = []
        self.active_endeffector = -1

        self.robot = robot

        self.endeffector_list = endeffector_list
        self.offset = offset
        self.axis = axis
        self.interpolation_steps = interpolation_steps
        self.sleep = sleep

        self.m_commands = m_commands
        self.t_commands = t_commands

        self.g_commands = [[] for _ in range(1000)]
        self.g_commands[54].append(lambda: self.g_54())
        self.g_commands[500].append(lambda: self.g_500())
        self.g_commands[17].append(lambda: self.g_17())
        self.g_commands[17].append(lambda: self.g_18())
        self.g_commands[17].append(lambda: self.g_19())

        if robot is not None:
            self.calibrate_tool()

        if endeffector_list is not None:
            self.active_endeffector = self.get_active_endeffector()

    def __iter__(self):
        self.operation = []
        self.index_operation = 0
        self.index_gcode = 0
        self.path = []
        return self

    def __next__(self):
        """Runs G-Code row by row. It is necessary to create the G-Code with
        the same structure as the readGcode(textfile) Method.

        Args:
            gcode (list): G-Code created from read_Gcode()

        Returns:
            None
        """

        if self.index_operation < len(self.operation):
            i = self.index_operation
            self.index_operation += 1
            self.operation[i]()

        elif self.index_gcode < len(self.gcode):
            self.operation = []
            self.index_operation = 0

            i = self.index_gcode
            cmd_type = self.gcode[i][0][0]
            cmd_int = self.gcode[i][0][1]

            self.last_point = np.array(self.new_point)
            self.last_or = np.array(self.new_or)

            if cmd_type == "M":
                self.create_elementary_actions(
                    m_commands=self.m_commands[cmd_int])

            if cmd_type == "T":
                self.create_elementary_actions(
                    t_commands=self.t_commands[cmd_int])

            elif cmd_type == "G" and cmd_int > 3:

                self.create_elementary_actions(
                    g_commands=cmd_int)

            self.index_gcode += 1

        else:
            raise StopIteration

    def create_elementary_actions(self, g_int=None, m_int=None, t_int=None):

        if g_int is not None:
            if g_int > 3:
                for actions in self.g_commands[g_int]:
                    self.operation.append(lambda: actions())
            else:
                path = self.create_path(self)
                self.set_path(self, path)

        elif m_int is not None:
            for actions in self.m_commands[m_int]:
                self.operation.append(lambda: actions())

        elif t_int is not None:
            for actions in self.t_commands[t_int]:
                self.operation.append(lambda: actions())

            self.operation.append(lambda: self.calibrate_tool())

        # Activation of the zero offset
    def g_54(self):
        self.offset = np.array([self.last_point, self.last_or])

        # Deactivation of the zero offset
    def g_500(self):
        self.offset = np.array([[0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])

        # Axis selelection for circular interpolation
    def g_17(self):
        self.axis = 2  # X-Y Axis

    def g_18(self):
        self.axis = 1  # X-Z Axis

    def g_19(self):
        self.axis = 0  # Y-Z Axis

    def create_path(self):

        cmd = self.gcode[self.index_gcode]
        g_com = cmd[0][1]

        variables = {'X': np.nan, 'Y': np.nan, 'Z': np.nan, 'A': np.nan,
                     'B': np.nan, 'C': np.nan, 'R': np.nan}

        for val in cmd:
            if val[0] in variables:
                variables[val[0]] = val[1]

        xyz_val = np.array([variables['X'], variables['Y'],
                            variables['Z']])
        abc_val = np.array([variables['A'], variables['B'],
                            variables['C']])
        r_val = variables['R']

        # Setting the new point considering offset
        self.new_point = np.array([0.0, 0.0, 0.0])
        for i, value in enumerate(xyz_val):
            if np.isnan(value):
                self.new_point[i] = self.last_point[i]
            else:
                self.new_point[i] = value + self.offset[0][i]

        # Setting the new orientation considering offset
        self.new_or = np.array([0.0, 0.0, 0.0])
        for i, value in enumerate(abc_val):
            if np.isnan(value):
                self.new_or[i] = self.last_or[i]
            else:
                self.new_or[i] = value + self.offset[1][i]

        # Building the Path if there is a linear G1 interpolation
        if g_com == 0:
            path = linear_interpolation(self.last_point,
                                        self.new_point,
                                        0)

        # Building the Path if there is a linear G1 interpolation
        if g_com == 1:
            path = linear_interpolation(self.last_point,
                                        self.new_point,
                                        self.interpolation_steps)

        # Building the path if there is a circular interpolation
        elif g_com in [2, 3]:
            if g_com == 2:
                path = circular_interpolation(self.last_point,
                                              self.new_point, r_val,
                                              self.interpolation_steps,
                                              self.axis, True)
            else:
                path = circular_interpolation(self.last_point,
                                              self.new_point, r_val,
                                              self.interpolation_steps,
                                              self.axis, False)

        orientation = p.getQuaternionFromEuler(self.new_or)
        path.orientations = np.transpose([orientation]
                                         * len(path.orientations[0]))

        return path

    def set_path(self):
        active = self.active_endeffector  # abbreviation

        for position, orientation, _ in self.path:
            if self.active_endeffector == -1:
                self.operation.append(
                    lambda i=position, j=orientation: self.robot.set_endeffector_pose(i, j))

            else:
                self.operation.append(
                    lambda i=position, j=orientation: self.endeffector_list[active].set_tool_pose(i, j))

    def get_active_endeffector(self):
        """Returns the index of the active endeffector of self.endeffector_list

        Args:
            None

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

    def calibrate_tool(self):
        """This method sets the current postion of the active Tool. This
        ensures a smooth transition between tool changes or setting the
        start point after launching the gcode class.

        Args:
            None

        Returns:
            None
        """
        self.active_endeffector = self.get_active_endeffector()
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
