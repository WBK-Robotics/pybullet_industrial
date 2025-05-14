import pybullet as p
from pybullet_industrial import RobotBase

JOINT_KEY = ('RA')


class GCodeLogger:
    """
    A class for logging G-code based on changes in robot state.

    Args:
        robot (RobotBase): The robot instance.
    """

    def __init__(self, robot: RobotBase):
        self.robot = robot
        self.current_robot_pose = {
            'X': 0.0, 'Y': 0.0,
            'Z': 0.0, 'A': 0.0,
            'B': 0.0, 'C': 0.0
        }
        # Build the current_joint_position dictionary dynamically using
        # the robot's moveable joints and assigning sequential keys.
        moveable_joints, _ = self.robot.get_moveable_joints()
        self.current_joint_position = {}
        for i, _ in enumerate(moveable_joints):
            key = f'{JOINT_KEY}{i+1}'
            self.current_joint_position[key] = 0.0

        self.g_code_robot_view = []
        self.g_code_joint_position = []

    @staticmethod
    def write_g_code(g_code: list, textfile: str,
                     translate: dict = None,
                     prefix: str = None,
                     postfix: str = None):
        """
        Write the given G-code commands to a text file, with optional
        prefix and postfix.

        Args:
            g_code (list): List of dictionaries representing G-code.
            textfile (str): Path to the file where G-code will be written.
            translate (dict, optional): Dictionary for translating specific
                commands (e.g., {'M11': '%@example_call'}).
                Defaults to None.
            prefix (str, optional): String to be written at the beginning.
                Defaults to None.
            postfix (str, optional): String to be written at the end.
                Defaults to None.
        """
        def format_value(value):
            """
            Format a numerical value for G-code output.

            - Uses fixed-point notation for scientific format.
            - Removes trailing zeros and decimal points for clean output.

            Args:
                value (float): The numerical value to format.

            Returns:
                str: Formatted string representation.
            """
            str_val = str(value)
            if 'e' in str_val.lower():
                str_val = f'{value:.10f}'
            if '.' in str_val:
                str_val = str_val.rstrip('0').rstrip('.')
            return str_val

        if translate is None:
            translate = {}

        with open(textfile, 'w') as file:
            if prefix is not None:
                file.write(prefix + '\n')
            for command in g_code:
                fixed_order = ['G', 'X', 'Y', 'Z', 'A', 'B', 'C']
                joint_keys = sorted(
                    [key for key in command
                     if key.startswith(JOINT_KEY) and key[len(JOINT_KEY):].isdigit()],
                    key=lambda k: int(k[len(JOINT_KEY):])
                )
                order = fixed_order + joint_keys
                line_items = []
                for key in order:
                    if key in command:
                        formatted_val = format_value(command[key])
                        if key.startswith('RA'):
                            line_items.append(f'{key}={formatted_val}')
                        else:
                            line_items.append(f'{key}{formatted_val}')
                for key in command:
                    if key not in order:
                        check_translate = key + str(command[key])
                        if check_translate in translate:
                            line_items.append(translate[check_translate])
                        else:
                            formatted_val = format_value(command[key])
                            line_items.append(f'{key}{formatted_val}')
                file.write(' '.join(line_items) + '\n')
            if postfix is not None:
                file.write(postfix + '\n')

    def update_g_code(self, only_changed_values=True):
        """
        Update both G-code logs (robot view and joint positions).

        Args:
            only_changed_values (bool): If True, only changed values
                are written to the G-code log. If False, all values
                are written regardless of change.
        """
        self.update_g_code_robot_view(only_changed_values)
        self.update_g_code_joint_position(only_changed_values)

    def update_g_code_robot_view(self, only_changed_values=True):
        """
        Update G-code for robot workspace coordinates.
        """
        self._update_g_code(self._get_robot_pose(),
                            self.current_robot_pose,
                            self.g_code_robot_view,
                            only_changed_values)

    def update_g_code_joint_position(self, only_changed_values=True):
        """
        Update G-code for joint positions.
        """
        self._update_g_code(self._get_joint_position(),
                            self.current_joint_position,
                            self.g_code_joint_position,
                            only_changed_values)

    def _update_g_code(self, new_pose, current_pose, g_code_list,
                       only_changed_values=True):
        """
        Update the G-code based on changes in pose.

        Args:
            new_pose (dict): New pose values.
            current_pose (dict): Current pose values.
            g_code_list (list): List to append the G-code line.
        """
        changed = [a != b for a, b in zip(
            new_pose.values(), current_pose.values())]
        if any(changed):
            current_pose.update(new_pose)
            g_line = {'G': 1}
            for key, value, val_change in zip(new_pose.keys(), new_pose.values(),
                                              changed):
                if not only_changed_values:
                    val_change = True
                if val_change:
                    g_line[key] = value
            g_code_list.append(g_line)

    def _get_robot_pose(self):
        """
        Get the current pose of the robot in workspace coordinates.

        Returns:
            dict: Robot pose in workspace coordinates.
        """
        pos, quat = self.robot.get_endeffector_pose()
        euler = p.getEulerFromQuaternion(quat)
        return {'X': pos[0], 'Y': pos[1],
                'Z': pos[2], 'A': euler[0],
                'B': euler[1], 'C': euler[2]}

    def _get_joint_position(self):
        """
        Get the current joint positions using the robot's
        get_joint_position() call and convert keys to a standard format
        using JOINT_KEY and sequential numbering (e.g., RA1, RA2, etc.).

        Returns:
            dict: Joint positions with keys in the form RA1, RA2, etc.
        """
        orig = self.robot.get_joint_position()
        moveable_joints, _ = self.robot.get_moveable_joints()
        new_pos = {}
        for i, name in enumerate(moveable_joints):
            pos = orig[name]
            joint_index = self.robot._joint_name_to_index[name]
            lower_lim = self.robot._lower_joint_limit[joint_index]
            upper_lim = self.robot._upper_joint_limit[joint_index]
            if pos > upper_lim:
                pos = upper_lim
            elif pos < lower_lim:
                pos = lower_lim
            key = f'{JOINT_KEY}{i+1}'
            new_pos[key] = pos
        return new_pos
