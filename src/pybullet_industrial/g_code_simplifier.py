import numpy as np


class GCodeSimplifier:
    """
    Methods are provided to simplify G-code.
    It is assumed that G-code is provided as a list of
    dictionaries.
    """

    @staticmethod
    def scale_g_code(g_code, scaling, keys):
        """
        Scales the values of specified keys in G-code commands.

        Parameters:
            g_code (list): List of G-code command dictionaries.
            scaling (float): Scaling factor.
            keys (iterable): Keys whose values are to be scaled.

        Returns:
            list: Scaled G-code commands.
        """
        for command in g_code:
            for key in keys:
                if key in command:
                    command[key] *= scaling
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
        start with 'RA'.

        Parameters:
            g_code (list): List of G-code command dictionaries.

        Returns:
            list: G-code commands with angles converted to radians.
        """
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith('RA'):
                    command[key] = np.radians(command[key])
        return g_code

    @staticmethod
    def convert_to_degrees(g_code):
        """
        Converts angles in G-code commands from radians to degrees.

        Conversion is applied to keys 'A', 'B', 'C' and those that
        start with 'RA'.

        Parameters:
            g_code (list): List of G-code command dictionaries.

        Returns:
            list: G-code commands with angles converted to degrees.
        """
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith('RA'):
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
