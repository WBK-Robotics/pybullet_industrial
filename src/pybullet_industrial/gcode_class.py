import pybullet as p
import pybullet_industrial as pi
import numpy as np
import time


class Gcode_class():

    def __init__(self, robot: pi.RobotBase = None,
                 endeffector_list: list = None,
                 m_commands: list = None,
                 t_commands: list = None, offset: np.array = None,
                 plane: int = None, interpolation_steps: int = None,
                 sleep: int = None):
        """Initialize a PathMover object with the provided parameters.

        Args:
            robot (pi.RobotBase): The robot to be moved.
            filename (str, optional): The name of a G-code file to read.
            endeffector_list (list, optional): List of end effectors to use.
            m_commands (list, optional): M-commands to execute.
            t_commands (list, optional): T-commands to execute.
            offset (np.array, optional): The offset for the path points.
            plane (int, optional): The plane for circular interpolation.
            interpolation_steps (int, optional): Number of interpolation steps.

        Returns:
            None
        """
        self.new_point = []
        self.new_or = []

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

        if plane is not None:
            self.plane = plane
        else:
            self.plane = 2  # X-Y Plane for the circular interpolation

        if interpolation_steps is not None:
            self.interpolation_steps = interpolation_steps
        else:
            self.interpolation_steps = 10

        if sleep is not None:
            self.sleep = sleep
        else:
            self.sleep = 0.0001

    def read_gcode(self, filename: str):
        """Reads G-Code row by row and saves the processed Data in
        a stacked List.
        Comments that start with % are ignored and all the other data is
        stored as it gets read in.
        Every Line in the G-Code resembles the same structure as the text file

        Args:
            filename (str): Source of information

        Returns:
            gcode
        """
        with open(filename) as f:
            gcode = []

            # Loop over the lines of the file
            while True:
                line = f.readline()

                # If the end of the file is reached, break the loop
                if not line:
                    break

                # Initialize a new line as a list
                new_line = []

                # If the line starts with "%", it is a comment line
                if line[0] == "%" or len(line) <= 1:
                    pass

                # Otherwise, split the line into its components and insert
                # them into the new line
                else:
                    # Split the line into its components
                    data = line.split()

                    # Loop over the components
                    for i in data:
                        # Determine the ID of the component
                        id = i[0]

                        # Extract the value of the component
                        val2 = float(i[1:])

                        if id in ["G", "M", "T"]:
                            # Insert the value into the corresponding
                            # column of the new line
                            new_line.append([id, int(val2)])
                        else:
                            new_line.append([id, val2])

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

            g_com, m_com, t_com = np.nan, np.nan, np.nan

            # Checking for the command and advising it to the value
            if cmd[0][0] == "G":
                g_com = cmd[0][1]
            elif cmd[0][0] == "M":
                m_com = cmd[0][1]
            elif cmd[0][0] == "T":
                t_com = cmd[0][1]

            x_val, y_val, z_val = np.nan, np.nan, np.nan
            a_val, b_val, c_val = np.nan, np.nan, np.nan
            r_val = np.nan

            for val in cmd:
                if val[0] == "X":
                    x_val = val[1]
                elif val[0] == "Y":
                    y_val = val[1]
                elif val[0] == "Z":
                    z_val = val[1]
                elif val[0] == "A":
                    a_val = val[1]
                elif val[0] == "B":
                    b_val = val[1]
                elif val[0] == "C":
                    c_val = val[1]
                elif val[0] == "R":
                    r_val = val[1]

            xyz_val = np.array([x_val, y_val, z_val])
            abc_val = np.array([a_val, b_val, c_val])

            # Checking for a G-command
            if not np.isnan(g_com):
                self.last_point = np.array(self.new_point)
                self.last_or = np.array(self.new_or)

                # Checking for interpolation commands
                if g_com == 1 or g_com == 0 or g_com == 2 or g_com == 3:

                    # Setting the new point considering offset
                    self.new_point = np.array([0.0, 0.0, 0.0])
                    c = 0

                    for n in xyz_val:

                        if np.isnan(n):
                            self.new_point[c] = self.last_point[c]

                        else:
                            self.new_point[c] = n + self.offset[0][c]

                        c += 1

                    # Setting the new orientation considering offset
                    self.new_or = np.array([0.0, 0.0, 0.0])
                    c = 0

                    for n in abc_val:

                        if np.isnan(n):
                            self.new_or[c] = self.last_or[c]

                        else:
                            self.new_or[c] = n + self.offset[1][c]

                        c += 1

                    # Moving the endeffector if there is a G0 Interpolation
                    if g_com == 0:
                        self.g0_interpolation()

                    # Moving the endeffector if ther is a G1 Interpolation
                    elif g_com == 1 or g_com == 2 or g_com == 3:
                        self.g123_interpolation(g_com, r_val)

                # Activation of the zero offset
                elif g_com == 54:
                    self.offset = np.array([self.last_point, self.last_or])

                # Deactivation of the zero offset
                elif g_com == 500:
                    self.offset = np.array([[0.0, 0.0, 0.0],
                                           [0.0, 0.0, 0.0]])

                # Plane selelection for circular interpolation
                elif g_com == 17:
                    self.plane = 2  # X-Y Plane

                elif g_com == 18:
                    self.plane = 1  # X-Z Plane

                elif g_com == 19:
                    self.plane = 0  # Y-Z Plane

            # Checking for a M-command
            elif not np.isnan(m_com):
                self.m_commands[m_com][0]()

            # Checking for a T-command
            elif not np.isnan(t_com):
                self.t_commands[t_com][0]()
                self.calibrate_tool()

    def g0_interpolation(self):
        """This method is used in the run_gcode() Method. The G0-interpolation
        sets the position of the endeffector or tool without running an
        Interpolation command

        Args:
            None
        Returns:
            None
        """
        orientation = p.getQuaternionFromEuler(self.new_or)
        e = self.active_endeffector

        if not self.active_endeffector == -1:

            for i in range(15):
                self.endeffector_list[e].set_tool_pose(self.new_point,
                                                       orientation)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(self.sleep)

                current_position = self.endeffector_list[e].get_tool_pose()[0]
                or_euler = p.getEulerFromQuaternion(
                    self.endeffector_list[e].get_tool_pose()[1])
                current_orientation = np.array(or_euler)

                position_error = np.linalg.norm(current_position
                                                - self.new_point)

                orientation_error = np.linalg.norm(current_orientation
                                                   - self.new_or)

                if position_error < 0.02 and orientation_error < 0.004:
                    break
        else:

            for i in range(20):
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
            path = pi.linear_interpolation(self.last_point,
                                           self.new_point,
                                           self.interpolation_steps)

        # Building the path if there is a circular interpolation
        elif g_com == 2 or g_com == 3:
            if g_com == 2:
                path = pi.circular_interpolation(self.last_point,
                                                 self.new_point, r_val,
                                                 self.interpolation_steps,
                                                 self.plane, True)
            else:
                path = pi.circular_interpolation(self.last_point,
                                                 self.new_point, r_val,
                                                 self.interpolation_steps,
                                                 self.plane, False)

        path.orientations = np.transpose([orientation]
                                         * len(path.orientations[0]))

        # Moving endeffector or the robot
        if not self.active_endeffector == -1:
            self.move_along_path(path)
        else:
            self.move_robot(path)

    def move_along_path(self, path: pi.ToolPath):
        """Moves the active endeffector along a designated Path.

        Args:
            path(pi.ToolPath): Array of points defining the path
        Returns:
            None
        """
        e = self.active_endeffector

        for positions, orientations, tool_path in path:

            for i in range(10):

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

    def move_robot(self, path: pi.ToolPath):
        """Moves the endeffector of the robot along the provided path.

        Args:
            path (pi.ToolPath): Array of points defining the path
        Returns:
            None
        """
        for positions, orientations, tool_path in path:

            for i in range(20):

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
        n = 0
        active_endeffector = -1

        if self.endeffector_list is not None:

            for i in self.endeffector_list:

                if i.is_coupled():
                    active_endeffector = n
                    break

                n = n + 1

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
        e = self.active_endeffector

        if not self.active_endeffector == -1:
            self.new_point = self.endeffector_list[e].get_tool_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                self.endeffector_list[e].get_tool_pose()[1])
            self.new_or = np.array(or_euler)

        else:
            self.new_point = self.robot.get_endeffector_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                self.robot.get_endeffector_pose()[1])
            self.new_or = np.array(or_euler)
