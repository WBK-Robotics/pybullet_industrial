import pybullet as p
from pybullet_industrial import RobotBase
from pybullet_industrial import ToolPath
from pybullet_industrial import linear_interpolation
from pybullet_industrial import circular_interpolation
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
        self.m_commands = m_commands
        self.t_commands = t_commands
        self.endeffector_list = endeffector_list
        self.offset = offset
        self.axis = axis
        self.interpolation_steps = interpolation_steps
        self.sleep = sleep

        if robot is not None:
            self.calibrate_tool()

        if endeffector_list is not None:
            self.active_endeffector = self.get_active_endeffector()

    def __iter__(self):
        self.index = 0
        self.wait = True
        self.tool_path_index = 0
        self.tool_path = []
        return self

    def __next__(self):
        """Runs G-Code row by row. It is necessary to create the G-Code with
        the same structure as the readGcode(textfile) Method.

        Args:
            gcode (list): G-Code created from read_Gcode()

        Returns:
            None
        """

        if self.index < len(self.gcode):
            cmd_type = self.gcode[self.index][0][0]
            i = self.index

            if self.wait:
                self.index += 1

            if cmd_type == "M":
                self.excecute_m_cmd(self.gcode[i])

            if cmd_type == "T":
                self.execute_t_cmd(self.gcode[i])

            elif cmd_type == "G":
                if self.tool_path_index < self.tool_path.__len__():
                    if self.tool_path_index == self.tool_path.__len__() - 1:
                        self.wait = True
                    self.play_toolpath(self.tool_path, self.tool_path_index)
                else:
                    self.excecute_g_cmd(self.gcode[i])

        else:
            raise StopIteration

    def excecute_m_cmd(self, cmd):
        self.m_commands[cmd[0][1]][0]()

    def execute_t_cmd(self, cmd):
        self.t_commands[cmd[0][1]][0]()
        self.calibrate_tool()

    def excecute_g_cmd(self, cmd):

        self.last_point = np.array(self.new_point)
        self.last_or = np.array(self.new_or)

        g_cmd_type = cmd[0][1]

        # Activation of the zero offset
        if g_cmd_type == 54:
            self.offset = np.array([self.last_point, self.last_or])

        # Deactivation of the zero offset
        elif g_cmd_type == 500:
            self.offset = np.array([[0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0]])

        # Axis selelection for circular interpolation
        elif g_cmd_type == 17:
            self.axis = 2  # X-Y Axis

        elif g_cmd_type == 18:
            self.axis = 1  # X-Z Axis

        elif g_cmd_type == 19:
            self.axis = 0  # Y-Z Axis

        elif g_cmd_type in [0, 1, 2, 3]:
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

            # Moving the endeffector if there is a G0 Interpolation
            if g_cmd_type == 0:
                self.g0_interpolation()

            # Moving the endeffector if ther is a G1 Interpolation
            elif g_cmd_type in [1, 2, 3]:
                self.g123_interpolation(g_cmd_type, r_val)

    def g0_interpolation(self):
        """The G0-interpolation sets the position of the endeffector
        or tool without running an interpolation command

        Args:
            None
        Returns:
            None
        """
        orientation = p.getQuaternionFromEuler(self.new_or)
        actv = self.active_endeffector  # abbreviation

        if self.active_endeffector == -1:
            return self.robot.set_endeffector_pose(self.new_point, orientation)

        else:
            return self.endeffector_list[actv].set_tool_pose(self.new_point,
                                                             orientation)

    def g123_interpolation(self, g_com, r_val):
        """This Mehtod is part of the run_gcode() Method. Depending on the
        G-Command it either runs a linear or circular interpolation and calls
        the move-methods depending if a tool is activated or not.

        Args:
            g_com: G-Command from the g_code File
            r_val: Radius for the circular interpolation

        Returns:
            None
        """
        orientation = p.getQuaternionFromEuler(self.new_or)

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

        path.orientations = np.transpose([orientation]
                                         * len(path.orientations[0]))

        self.tool_path = path
        self.tool_path_index = 0
        self.wait = False

        self.play_toolpath(self.tool_path, 0)

    def play_toolpath(self, path: ToolPath, index: int):
        """Moves the active endeffector along a designated Path.

        Args:
            path(ToolPath): Array of points defining the path
        Returns:
            None
        """
        positions = path.positions[index]
        orientations = path.orientations[index]
        self.tool_path_index += 1

        if self.active_endeffector == -1:
            actv = self.active_endeffector  # abbreviation
            return self.endeffector_list[actv].set_tool_pose(
                positions, orientations)
        else:
            return self.robot.set_endeffector_pose(positions, orientations)

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
