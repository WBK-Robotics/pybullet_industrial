import numpy as np

JOINT_KEY = ('RA')


class GCodeSimplifier:
    """
    Methods are provided to simplify G-code.
    It is assumed that G-code is provided as a list of dicts.
    """

    @staticmethod
    def round_pose_values(g_code, decimals_xyz: int = 4,
                          decimals_abc: int = 4):
        """
        Rounds Cartesian and angular values in G-code commands.

        Cartesian coordinates (X, Y, Z) are rounded to decimals_xyz
        decimal places. Angular coordinates (A, B, C) are rounded
        to decimals_abc decimal places.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            decimals_xyz (int): Decimal places for Cartesian coordinates.
            decimals_abc (int): Decimal places for angular coordinates.

        Returns:
            list: G-code commands with rounded values.
        """
        for g_code_line in g_code:
            for key in g_code_line:
                if key in ['X', 'Y', 'Z']:
                    g_code_line[key] = round(g_code_line[key], decimals_xyz)
                elif key in ['A', 'B', 'C']:
                    g_code_line[key] = round(g_code_line[key], decimals_abc)
        return g_code

    @staticmethod
    def round_joint_positions(g_code, decimals_angles: int = 5):
        """
        Rounds joint positions in G-code commands using the JOINT_KEY
        prefix.

        For each key in the command, if the key starts with JOINT_KEY
        and the remainder consists only of digits, the corresponding
        value is rounded to decimals_angles decimal places.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            decimals_angles (int): Decimal places for joint positions.

        Returns:
            list: G-code commands with rounded joint positions.
        """
        for command in g_code:
            for key in list(command.keys()):
                if (key.startswith(JOINT_KEY) and
                        key[len(JOINT_KEY):].isdigit()):
                    command[key] = round(command[key], decimals_angles)
        return g_code

    @staticmethod
    def scale_g_code(g_code, scale_factor, keys):
        """
        Scales the values of specified keys in G-code commands.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            scale_factor (float): Scaling factor.
            keys (iterable): Keys whose values are to be scaled.

        Returns:
            list: Scaled G-code commands.
        """
        for command in g_code:
            for key in keys:
                if key in command:
                    command[key] *= scale_factor
        return g_code

    @staticmethod
    def add_offset_to_g_code(g_code, offset_dict):
        """
        Adds offsets to values of specified keys in G-code commands.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            offset_dict (dict): Dictionary of offsets. Keys correspond
                to command keys and values to offset amounts.

        Returns:
            list: G-code commands with offsets added.
        """
        for command in g_code:
            for key in offset_dict:
                if key in command:
                    command[key] += offset_dict[key]
        return g_code

    @staticmethod
    def convert_to_radians(g_code):
        """
        Converts angles in G-code commands from degrees to radians.

        Conversion is applied to keys 'A', 'B', 'C' and those that
        start with JOINT_KEY.

        Parameters:
            g_code (list): List of G-code command dictionaries.

        Returns:
            list: G-code commands with angles converted to radians.
        """
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith(JOINT_KEY):
                    command[key] = np.radians(command[key])
        return g_code

    @staticmethod
    def convert_to_degrees(g_code):
        """
        Converts angles in G-code commands from radians to degrees.

        Conversion is applied to keys 'A', 'B', 'C' and those that
        start with JOINT_Key.

        Parameters:
            g_code (list): List of G-code command dictionaries.

        Returns:
            list: G-code commands with angles converted to degrees.
        """
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith(JOINT_KEY):
                    command[key] = np.degrees(command[key])
        return g_code

    @staticmethod
    def apply_feedrate(g_code, feedrate):
        """
        Applies the given feedrate to G-code commands.

        The feedrate is applied only to movement commands (G-code 1).

        Parameters:
            g_code (list): List of G-code command dictionaries.
            feedrate (float): Feedrate value to be applied.

        Returns:
            list: G-code commands with the feedrate applied.
        """
        for g_code_line in g_code:
            if ('G' in g_code_line and g_code_line['G'] == 1 and
                    feedrate is not None):
                g_code_line['F'] = feedrate
        return g_code

    @staticmethod
    def skip_command(g_code, skip_commands):
        """
        Removes specified key-value pairs from G-code commands.

        Commands are filtered by deleting key-value pairs that match
        those in the skip_commands dictionary. The command is added
        to the list only if non-empty.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            skip_commands (dict): Dictionary of commands to be skipped.

        Returns:
            list: Filtered G-code commands.
        """
        filtered_g_code = []
        for command in g_code:
            for key in list(skip_commands.keys()):
                if key in command and command[key] == skip_commands[key]:
                    del command[key]
            if command:
                filtered_g_code.append(command)
        return filtered_g_code
