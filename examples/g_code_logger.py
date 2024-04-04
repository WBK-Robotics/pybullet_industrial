import pybullet as p
from pybullet_industrial import RobotBase


class GCodeLogger:
    """
    A class for logging G-code based on changes in robot state.

    Args:
        robot (RobotBase): The robot instance.
        round_decimal (int, optional): Number of decimal places to round values. Defaults to 4.
    """

    def __init__(self, robot: RobotBase, round_decimal: int = 4):

        self.robot = robot
        self.round_decimals = round_decimal
        self.current_robot_pose = {'X': 0.0, 'Y': 0.0,
                                   'Z': 0.0, 'A': 0.0, 'B': 0.0, 'C': 0.0}
        self.current_joint_position = {'RA1': 0.0, 'RA2': 0.0, 'RA3': 0.0,
                                       'RA4': 0.0, 'RA5': 0.0, 'RA6': 0.0}
        self.g_code_robot_view = []
        self.g_code_joint_position = []

    @staticmethod
    def write_g_code(g_code: list, textfile: str):
        """
        Write the given G-code commands to a text file.

        Args:
            g_code (list): List of dictionaries representing G-code commands.
            textfile (str): Path to the text file where G-code will be written.
        """
        with open(textfile, 'w') as file:
            for command in g_code:
                # Define the order in which keys should be written
                order = ['G', 'X', 'Y', 'Z', 'A', 'B', 'C',
                         'RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']
                # Construct the line by joining key-value pairs
                line = ' '.join(f'{key}{value}' for key,
                                value in command.items() if key in order)
                file.write(line + '\n')

    def get_robot_pose(self):
        """
        Get the current pose of the robot in Cartesian coordinates.

        Returns:
            dict: Current robot pose in Cartesian coordinates.
        """
        position = self.robot.get_endeffector_pose()[0]
        orientation_quaternion = self.robot.get_endeffector_pose()[1]
        orientation = p.getEulerFromQuaternion(orientation_quaternion)

        return {'X': position[0], 'Y': position[1], 'Z': position[2], 'A': orientation[0], 'B': orientation[1], 'C': orientation[2]}

    def update(self):
        """
        Update the G-code based on changes in robot state.
        """
        self.update_g_code_robot_view()
        self.update_g_code_joint_position()

    def add_g_code_line(self, g_code_line):
        """
        Add a G-code line to both robot view and joint position lists.

        Args:
            g_code_line (dict): G-code line to add.
        """
        self.g_code_joint_position.append(g_code_line)
        self.g_code_robot_view.append(g_code_line)

    def _round_values(self, pose_dict):
        """
        Round the values in a dictionary.

        Args:
            pose_dict (dict): Dictionary containing values to be rounded.

        Returns:
            dict: Dictionary with rounded values.
        """
        return {key: round(value, self.round_decimals) for key, value in pose_dict.items()}

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

            # Skip adding G: 1 for g_code_joint_position
            if g_code_list is not self.g_code_joint_position:
                g_code_line['G'] = 1

            for key, value, changed in zip(new_pose.keys(), new_pose.values(), changed_values):
                if changed:
                    g_code_line[key] = value

            g_code_list.append(g_code_line)

    def update_g_code_tcp(self):
        """
        Update G-code related to robot TCP.
        """
        new_tcp_pose = self._round_values(self.get_tcp_pose())
        self._update_g_code(new_tcp_pose, self._round_values(
            self.current_tcp_pose), self.g_code_tcp)

    def update_g_code_robot_view(self):
        """
        Update G-code related to robot Cartesian coordinates.
        """
        new_robot_pose = self._round_values(self.get_robot_pose())
        self._update_g_code(new_robot_pose, self._round_values(
            self.current_robot_pose), self.g_code_robot_view)

    def get_joint_position(self):
        """
        Get the current joint positions of the robot.

        Returns:
            dict: Current joint positions.
        """
        joint_states = self.robot.get_joint_state()
        joint_position = {}

        for joint_name, joint_data in joint_states.items():
            new_joint_name = 'RA' + joint_name[1]  # Replace 'Q' with 'N'
            joint_position[new_joint_name] = joint_data['position']

        return joint_position

    def update_g_code_joint_position(self):
        """
        Update G-code related to joint positions.
        """
        new_joint_position = self._round_values(self.get_joint_position())
        self._update_g_code(new_joint_position, self._round_values(
            self.current_joint_position), self.g_code_joint_position)
        current_g_code_line = self.g_code_joint_position[-1]
        current_g_code_line['G'] = 1
