import pybullet as p
from pybullet_industrial import RobotBase



class GCodeLogger:
    """
    A class for logging G-code based on changes in robot state.

    Args:
        robot (RobotBase): The robot instance.
    """

    def __init__(self, robot: RobotBase):

        self.robot = robot
        self.current_robot_pose = {'X': 0.0, 'Y': 0.0,
                                   'Z': 0.0, 'A': 0.0, 'B': 0.0, 'C': 0.0}
        self.current_joint_position = {'RA1': 0.0, 'RA2': 0.0, 'RA3': 0.0,
                                       'RA4': 0.0, 'RA5': 0.0, 'RA6': 0.0}
        self.g_code_robot_view = []
        self.g_code_joint_position = []

    @staticmethod
    def write_g_code(g_code: list, textfile: str,
                     translate: dict = None,
                     prefix: str = None,
                     postfix: str = None):
        """
        Write the given G-code commands to a text file, with optional prefix and postfix.

        Args:
            g_code (list): List of dictionaries representing G-code commands.
            textfile (str): Path to the text file where G-code will be written.
            translate (dict, optional): Dictionary for translating specific commands
                                        ({'M11': '%@example_call'}). Defaults to None.
            prefix (str, optional): String to be written at the beginning of the file. Defaults to None.
            postfix (str, optional): String to be written at the end of the file. Defaults to None.
        """
        def format_value(value):
            # Ensure the value is not in scientific notation and remove trailing zeros
            # formatted_value = f'{value:.15f}' if isinstance(
            #     value, float) else str(value)
            string_value = str(value)
            if 'e' in string_value.lower():
                formatted_value = f'{value:.10f}'
            else:
                formatted_value = string_value


            return formatted_value.rstrip('0').rstrip('.') if '.' in formatted_value else formatted_value

        with open(textfile, 'w') as file:
            # Write the prefix if it exists
            if prefix is not None:
                file.write(prefix + '\n')

            for command in g_code:
                order = ['G', 'X', 'Y', 'Z', 'A', 'B', 'C',
                         'RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']
                line_items = []

                # Write keys in the specified order
                for key in order:
                    if key in command:
                        formatted_value = format_value(command[key])
                        line_items.append(f'{key}={formatted_value}' if key.startswith(
                            'RA') else f'{key}{formatted_value}')

                # Write keys that are not in the specified order
                for key in command:
                    if key not in order:
                        check_translate = key + str(command[key])
                        if check_translate in translate:
                            line_items.append(translate[check_translate])
                        else:
                            formatted_value = format_value(command[key])
                            line_items.append(f'{key}{formatted_value}')

                file.write(' '.join(line_items) + '\n')

            # Write the postfix if it exists
            if postfix is not None:
                file.write(postfix + '\n')

    def update_g_code(self):
        """
        Update both of the G-codes based on changes in robot state.
        """
        self.update_g_code_robot_view()
        self.update_g_code_joint_position()

    def update_g_code_robot_view(self):
        """
        Update G-code related to robot Cartesian coordinates.
        """
        self._update_g_code(self._get_robot_pose(),
                            self.current_robot_pose, self.g_code_robot_view)

    def update_g_code_joint_position(self):
        """
        Update G-code related to joint positions.
        """
        self._update_g_code(self._get_joint_position(),
                            self.current_joint_position,
                            self.g_code_joint_position)

    def _update_g_code(self, new_pose, current_pose, g_code_list):
        """
        Update the G-code based on changes in pose.

        Args:
            new_pose (dict): New pose values.
            current_pose (dict): Current pose values.
            g_code_list (list): List to append the G-code line.
        """
        changed_values = [a != b for a, b in zip(
            new_pose.values(), current_pose.values())]

        if any(changed_values):
            current_pose.update(new_pose)  # Update current pose in-place

            # Add G-code line with changed values
            g_code_line = {}
            g_code_line['G'] = 1

            for key, value, changed in zip(new_pose.keys(), new_pose.values(), changed_values):
                #if changed:
                g_code_line[key] = value

            g_code_list.append(g_code_line)

    def _get_robot_pose(self):
        """
        Get the current pose of the robot in Cartesian coordinates.

        Returns:
            dict: Current robot pose in Cartesian coordinates.
        """
        position = self.robot.get_endeffector_pose()[0]
        orientation_quaternion = self.robot.get_endeffector_pose()[1]
        orientation = p.getEulerFromQuaternion(orientation_quaternion)

        return {'X': position[0], 'Y': position[1], 'Z': position[2], 'A': orientation[0], 'B': orientation[1], 'C': orientation[2]}

    def _get_joint_position(self):
        """
        Get the current joint positions of the robot.

        Returns:
            dict: Current joint positions.
        """
        joint_states = self.robot.get_joint_state()
        joint_position = {}

        for joint_name, joint_data in joint_states.items():
            new_joint_name = 'RA' + joint_name[1]  # Replace 'Q' with 'RA'
            joint_number = self.robot._joint_name_to_index[joint_name]

            # Adjust joint position if it exceeds joint limits
            lower_limit = self.robot._lower_joint_limit[joint_number]
            upper_limit = self.robot._upper_joint_limit[joint_number]
            joint_position[new_joint_name] = joint_data['position']

            # Check if joint position exceeds limits
            if joint_position[new_joint_name] > upper_limit:
                joint_position[new_joint_name] = upper_limit
            elif joint_position[new_joint_name] < lower_limit:
                joint_position[new_joint_name] = lower_limit

        return joint_position
