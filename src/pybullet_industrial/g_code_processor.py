import time
import pybullet as p
from pybullet_industrial import RobotBase
from pybullet_industrial import ToolPath
from pybullet_industrial import linear_interpolation
from pybullet_industrial import circular_interpolation
import numpy as np


class GCodeProcessor:

    def __init__(self, robot: RobotBase = None,
                 endeffector_list: list = None,
                 m_commands: list = None,
                 t_commands: list = None, offset: np.array = None,
                 axis: int = 2, interpolation_steps: int = 10,
                 sleep: int = 0.0001):
        """Initialize a PathMover object with the provided parameters.

        Args:
            robot (pi.RobotBase, optional: The robot which is controlled.
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
        self.new_point = []
        self.new_or = []
        self.last_point = []
        self.last_or = []

        if m_commands is not None:
            self.m_commands = m_commands

        if t_commands is not None:
            self.t_commands = t_commands

        if endeffector_list is not None:
            self.endeffector_list = endeffector_list
            self.active_endeffector = self.get_active_endeffector()
        else:
            self.endeffector_list = None
            self.active_endeffector = -1

        if robot is not None:
            self.robot = robot
            self.calibrate_tool()

        if offset is not None:
            self.offset = offset
        else:
            self.offset = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        if axis is not None:
            self.axis = axis
        else:
            self.axis = 2  # X-Y Axis for the circular interpolation

        if interpolation_steps is not None:
            self.interpolation_steps = interpolation_steps
        else:
            self.interpolation_steps = 10

        if sleep is not None:
            self.sleep = sleep
        else:
            self.sleep = 0.0001

    @staticmethod
    def read_gcode(filename: str):
        """Reads G-Code row by row and saves the processed Data in
        a List.
        Comments that start with % are ignored and all the other data is
        stored as it gets read in.
        Every Line in the G-Code resembles the same structure as the text file

        Args:
            filename (str): Source of information

        Returns:
            gcode
        """
        with open(filename, encoding='utf-8') as f:
            gcode = []

            # Loop over the lines of the file
            for line in f.readlines():

                # If the end of the file is reached, break the loop
                if not line:
                    break

                # Initialize a new line as a list
                new_line = []

                # Read in G-Code if line is not a comment and not empty
                if line[0] != "%" and len(line) > 1:

                    # Split the line into its components
                    data = line.split()

                    # Loop over the components
                    for i in data:
                        # Determine the ID of the component
                        id_val = i[0]

                        # Extract the value of the component
                        val2 = float(i[1:])

                        if id_val in ["G", "M", "T"]:
                            # Insert the value into the corresponding
                            # column of the new line
                            new_line.append([id_val, int(val2)])
                        else:
                            new_line.append([id_val, val2])

                    # Add the finished line to the list
                    gcode.append(new_line)

            return gcode

    def run_gcode(self, gcode: list):
        """Runs G-Code row by row. It is necessary to create the G-Code with
        the same structure as the readGcode(textfile) Method.

        Args:
            gcode (list): G-Code created from read_Gcode()

        Returns:
            None
        """
        # Runs the information out of the input_array
        for cmd in gcode:

            cmd_type = cmd[0][0]

            if cmd_type == "M":
                self.excecute_m_cmd(cmd)

            if cmd_type == "T":
                self.execute_t_cmd(cmd)

            elif cmd_type == "G":
                self.excecute_g_cmd(cmd)

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

        # Plane selelection for circular interpolation
        elif g_cmd_type == 17:
            self.plane = 2  # X-Y Plane

        elif g_cmd_type == 18:
            self.plane = 1  # X-Z Plane

        elif g_cmd_type == 19:
            self.plane = 0  # Y-Z Plane

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
            for _ in range(20):
                self.robot.set_endeffector_pose(self.new_point, orientation)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(self.sleep)

                current_position = self.robot.get_endeffector_pose()[0]
                or_euler = p.getEulerFromQuaternion(
                    self.robot.get_endeffector_pose()[1])
                current_orientation = np.array(or_euler)

                position_error = np.linalg.norm(current_position -
                                                self.new_point)
                orientation_error = np.linalg.norm(current_orientation -
                                                   self.new_or)

                if position_error < 0.02 and orientation_error < 0.004:
                    break

        else:
            for _ in range(20):
                self.endeffector_list[actv].set_tool_pose(self.new_point,
                                                          orientation)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(self.sleep)

                current_position = self.endeffector_list[actv].get_tool_pose()[
                    0]
                or_euler = p.getEulerFromQuaternion(
                    self.endeffector_list[actv].get_tool_pose()[1])
                current_orientation = np.array(or_euler)

                position_error = np.linalg.norm(current_position
                                                - self.new_point)

                orientation_error = np.linalg.norm(current_orientation
                                                   - self.new_or)

                if position_error < 0.02 and orientation_error < 0.004:
                    break

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
                                              self.plane, True)
            else:
                path = circular_interpolation(self.last_point,
                                              self.new_point, r_val,
                                              self.interpolation_steps,
                                              self.plane, False)

        path.orientations = np.transpose([orientation]
                                         * len(path.orientations[0]))

        # Moving endeffector or the robot
        if self.active_endeffector == -1:
            self.move_robot(path)
        else:
            self.move_endeffector(path)

    def move_endeffector(self, path: ToolPath):
        """Moves the active endeffector along a designated Path.

        Args:
            path(ToolPath): Array of points defining the path
        Returns:
            None
        """
        e = self.active_endeffector

        for positions, orientations, _ in path:

            for _ in range(20):

                self.endeffector_list[e].set_tool_pose(positions, orientations)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(self.sleep)

                current_position = self.endeffector_list[e].get_tool_pose()[0]
                current_orientation = np.array(p.getEulerFromQuaternion(
                    self.endeffector_list[e].get_tool_pose()[1]))

                position_error = np.linalg.norm(current_position-positions)
                orientations_euler = np.array(
                    p.getEulerFromQuaternion(orientations))
                orientation_error = np.linalg.norm(current_orientation -
                                                   orientations_euler)

                if position_error < 0.02 and orientation_error < 0.004:
                    break

    def move_robot(self, path: ToolPath):
        """Moves the endeffector of the robot along the provided path.

        Args:
            path (ToolPath): Array of points defining the path
        Returns:
            None
        """
        for positions, orientations, _ in path:

            for _ in range(20):

                self.robot.set_endeffector_pose(positions, orientations)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(self.sleep)

                current_position = self.robot.get_endeffector_pose()[0]
                current_orientation = np.array(p.getEulerFromQuaternion(
                    self.robot.get_endeffector_pose()[1]))

                position_error = np.linalg.norm(current_position-positions)
                orientations_euler = np.array(
                    p.getEulerFromQuaternion(orientations))
                orientation_error = np.linalg.norm(current_orientation -
                                                   orientations_euler)

                if position_error < 0.02 and orientation_error < 0.004:
                    break

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
