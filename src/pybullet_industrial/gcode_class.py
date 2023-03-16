import pybullet as p
import pybullet_industrial as pi
import numpy as np
import time


def move_along_path(endeffector: pi.EndeffectorTool,
                    path: pi.ToolPath, stop=False):
    """Move a designated robot along the provided path.

    Args:
        robot (pi.RobotBase): Robot to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop
        at the end of the movement.
    """
    for positions, orientations, tool_path in path:

        for i in range(10):
            endeffector.set_tool_pose(positions, orientations)
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.0001)
            current_position = endeffector.get_tool_pose()[0]
            current_orientation = np.array(p.getEulerFromQuaternion
                                           (endeffector.get_tool_pose()[1]))
            position_error = np.linalg.norm(current_position-positions)
            orientations_euler = np.array(
                p.getEulerFromQuaternion(orientations))
            orientation_error = np.linalg.norm(current_orientation -
                                               orientations_euler)

            if position_error < 0.02 and orientation_error < 0.004:
                break


def move_robot(robot: pi.RobotBase, path: pi.ToolPath, stop=False):
    """Move a designated robot along the provided path.

    Args:
        robot (pi.RobotBase): Robot to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop
        at the end of the movement.
    """
    for positions, orientations, tool_path in path:
        for i in range(20):
            robot.set_endeffector_pose(positions, orientations)
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.0001)
            current_position = robot.get_endeffector_pose()[0]
            current_orientation = np.array(p.getEulerFromQuaternion(
                robot.get_endeffector_pose()[1]))
            position_error = np.linalg.norm(current_position-positions)
            orientations_euler = np.array(
                p.getEulerFromQuaternion(orientations))
            orientation_error = np.linalg.norm(current_orientation -
                                               orientations_euler)

            if position_error < 0.02 and orientation_error < 0.004:
                break


def get_active_endeffector(endeffector_list: list):
    """Get the index of the active endeffector in a list of endeffectors.

    Args:
        endeffector_list (list): List of pi.EndeffectorTool objects.

    Returns:
        int: Index of the active endeffector, or -1 if no
        endeffectors are coupled.
    """
    n = 0
    active_endeffector = -1
    for i in endeffector_list:
        if i.is_coupled():
            active_endeffector = n
            break
        n = n + 1
    return active_endeffector


class Gcode_class():
    def __init__(self, filename=None, robot: pi.RobotBase = None,
                 endeffector_list=None, m_befehle=None, t_befehle=None):
        if filename is not None and robot is not None:
            gcode = self.read_gcode(filename)
            self.run_gcode(gcode, robot, endeffector_list,
                           m_befehle, t_befehle)

    def read_gcode(self, filename):
        """Reads G-Code row by row and saves the processed Data in
        a stacked List.
        Comments that start with % are ignored and all the other data is
        stored as it gets read in.
        Every Line in the G-Code resembles the same structure as the text file
        Args:
            filename (str or file-like object): Source of information

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

    def run_gcode(self, gcode: list, robot: pi.RobotBase,
                  endeffector_list: list = None, m_commands: list = None,
                  t_commands: list = None):
        """Runs G-Code row by row. It is necessary to create the G-Code with
        the same structure as the readGcode(textfile) Method.
        Args:
            gcode (list): G-Code created from read_Gcode()
            robot (pi.RobotBase): Operating Robot
            endeffector_list (list): Existing endeffector Tools
            m_commands (list): List which holds methods with all kinds of
            actions
            t_commands (list): List which holds methods with the corresponding
            tool to change
        Returns:
            None
        """

        if m_commands is not None:
            self.m_commands = m_commands
        if t_commands is not None:
            self.t_commands = t_commands
        if endeffector_list is not None:
            self.active_endeffector = get_active_endeffector(endeffector_list)
        else:
            self.active_endeffector = -1

        if not self.active_endeffector == -1:
            e = self.active_endeffector
            new_point = endeffector_list[e].get_tool_pose()[0]
            new_or = np.array(p.getEulerFromQuaternion(
                endeffector_list[self.active_endeffector].get_tool_pose()[1]))
        else:
            new_point = robot.get_endeffector_pose()[0]
            new_or = np.array(p.getEulerFromQuaternion(
                robot.get_endeffector_pose()[1]))

        # Default Values for Running the G-Code
        plane = 2  # X-Y Plane for the circular interpolation -G2 & -G3
        offset = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
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
                last_point = np.array(new_point)
                last_or = np.array(new_or)

                # Checking for interpolation commands
                if g_com == 1 or g_com == 0 or g_com == 2 or g_com == 3:
                    # Setting the new point considering offset
                    new_point = np.array([0.0, 0.0, 0.0])
                    counter = 0
                    for n in xyz_val:
                        if np.isnan(n):
                            new_point[counter] = last_point[counter]
                        else:
                            new_point[counter] = n + offset[0][counter]
                        counter = counter + 1
                    # Setting the new orientation considering offset
                    new_or = np.array([0.0, 0.0, 0.0])
                    counter = 0
                    for n in abc_val:
                        if np.isnan(n):
                            new_or[counter] = last_or[counter]
                        else:
                            new_or[counter] = n + offset[1][counter]
                        counter = counter + 1
                    orientation = p.getQuaternionFromEuler(new_or)
                    # Moving the endeffector if there is a G0 Interpolation
                    if g_com == 0:
                        self.g0_interpolation(robot, endeffector_list,
                                              new_point, new_or, orientation)
                    # Moving the endeffector if ther is a G1 Interpolation
                    elif g_com == 1 or g_com == 2 or g_com == 3:
                        self.g123_interpolation(robot, endeffector_list,
                                                g_com, r_val, plane,
                                                last_point, new_point,
                                                orientation)
                # Activation of the zero offset
                elif g_com == 54:
                    offset = np.array([last_point, last_or])
                # Deactivation of the zero offset
                elif g_com == 500:
                    offset = np.array([[0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0]])
                # Plane selelection for circular interpolation
                elif g_com == 17:
                    plane = 2  # X-Y Plane
                elif g_com == 18:
                    plane = 1  # X-Z Plane
                elif g_com == 19:
                    plane = 0  # Y-Z Plane
            # Checking for a M-command
            elif not np.isnan(m_com):
                self.m_commands[m_com][0]()

            # Checking for a T-command
            elif not np.isnan(t_com):
                self.t_commands[t_com][0]()
                new_point = self.calibrate_tool(robot, endeffector_list)[0]
                new_or = self.calibrate_tool(robot, endeffector_list)[1]

    def g0_interpolation(self, robot: pi.RobotBase, endeffector_list: list,
                         new_point, new_or, orientation):
        e = self.active_endeffector
        if not self.active_endeffector == -1:
            for i in range(15):
                endeffector_list[
                    self.active_endeffector].set_tool_pose(new_point,
                                                           orientation)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(0.0001)
                current_position = endeffector_list[e].get_tool_pose()[0]
                or_euler = p.getEulerFromQuaternion(
                    endeffector_list[e].get_tool_pose()[1])
                current_orientation = np.array(or_euler)
                position_error = np.linalg.norm(current_position - new_point)
                orientation_error = np.linalg.norm(current_orientation -
                                                   new_or)
                if position_error < 0.02 and orientation_error < 0.004:
                    break
        else:
            for i in range(20):
                robot.set_endeffector_pose(new_point, orientation)
                for _ in range(10):
                    p.stepSimulation()
                    time.sleep(0.0001)
                current_position = robot.get_endeffector_pose()[0]
                or_euler = p.getEulerFromQuaternion(
                    robot.get_endeffector_pose()[1])
                current_orientation = np.array(or_euler)
                position_error = np.linalg.norm(current_position-new_point)
                orientation_error = np.linalg.norm(current_orientation -
                                                   new_or)
                if position_error < 0.02 and orientation_error < 0.004:
                    break

    def g123_interpolation(self, robot: pi.RobotBase, endeffector_list, g_com,
                           r_val, plane, last_point, new_point, orientation):
        # Building the Path if there is a linear G1 interpolation
        if g_com == 1:
            path = pi.linear_interpolation(np.array(last_point),
                                           np.array(new_point), 10)
            path.orientations = np.transpose([orientation]
                                             * len(path.orientations[0]))

        # Building the path if there is a circular interpolation
        elif g_com == 2 or g_com == 3:
            if g_com == 2:
                path = pi.circular_interpolation(np.array(last_point),
                                                 np.array(new_point), r_val,
                                                 10, plane, True)
            else:
                path = pi.circular_interpolation(np.array(last_point),
                                                 np.array(new_point),
                                                 r_val, 10, plane, False)
            path.orientations = np.transpose([orientation]
                                             * len(path.orientations[0]))

        # Moving endeffector or the robot
        if not self.active_endeffector == -1:
            move_along_path(endeffector_list[self.active_endeffector], path)
        else:
            move_robot(robot, path)

    def calibrate_tool(self, robot: pi.RobotBase, endeffector_list):
        self.active_endeffector = get_active_endeffector(endeffector_list)
        e = self.active_endeffector
        if not self.active_endeffector == -1:
            new_point = endeffector_list[e].get_tool_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                endeffector_list[e].get_tool_pose()[1])
            new_or = np.array(or_euler)
        else:
            new_point = robot.get_endeffector_pose()[0]
            or_euler = p.getEulerFromQuaternion(
                robot.get_endeffector_pose()[1])
            new_or = np.array(or_euler)
        return [new_point, new_or]
