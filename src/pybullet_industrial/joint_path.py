import numpy as np


class JointPath:
    """A class for representing joint paths with joint configurations,
       joint order, and tool activations.

    Args:
        joint_values (np.array): A 2D array where each row represents joint
                                 values for a specific joint and each column
                                 represents a pose along the path.
        joint_order (list): A list defining the order of the joints
                            corresponding to the rows in `joint_values`.
        tool_activations (np.array, optional): A 1D array with boolean values
                                               indicating whether a tool is
                                               active at a given path pose.
                                               Defaults to None, meaning the
                                               tool is always inactive.

    Raises:
        ValueError: If `tool_activations` and the number of path poses
                    (columns in `joint_values`) have different lengths.
        ValueError: If the number of rows in `joint_values` does not match
                    the length of `joint_order`.
    """

    def __init__(self, joint_values: np.array, joint_order: list,
                 tool_activations: np.array = None):
        # Validate that joint values match the joint order length
        if joint_values.shape[0] != len(joint_order):
            raise ValueError(
                f"The number of rows in `joint_values` "
                f"({joint_values.shape[0]}) must match the length of "
                f"`joint_order` ({len(joint_order)})."
            )

        # Initialize joint values and order
        self.joint_values = joint_values
        self.joint_order = list(joint_order)  # ensure it's always a list

        # Initialize tool activations or set default values
        if tool_activations is None:
            self.tool_activations = np.zeros((joint_values.shape[1],),
                                             dtype=bool)
        else:
            if len(tool_activations) != joint_values.shape[1]:
                raise ValueError(
                    "The number of tool activations must match the number of "
                    "path poses (columns in `joint_values`)."
                )
            self.tool_activations = tool_activations

    def offset(self, offsets: dict):
        """Translates the joint values by adding specified offsets.

        Args:
            offsets (dict): A dictionary with joint names as keys and
                            offset values as values.

        Example:
            offsets = {'q1': 0.1, 'q2': -0.05}
        """
        for i, joint_name in enumerate(self.joint_order):
            if joint_name in offsets:
                self.joint_values[i] += offsets[joint_name]

    def append(self, joint_path):
        """Appends a given JointPath object to the end of this path.

        Args:
            joint_path (JointPath): Another JointPath object to append.

        Raises:
            ValueError: If the joint orders of the two JointPath objects are
                        not the same.
        """
        if self.joint_order != joint_path.joint_order:
            raise ValueError(
                "Joint orders must match to append JointPath objects."
            )

        self.joint_values = np.append(self.joint_values,
                                      joint_path.joint_values, axis=1)
        self.tool_activations = np.append(self.tool_activations,
                                          joint_path.tool_activations)

    def prepend(self, joint_path):
        """Prepends a given JointPath object to the start of this path.

        Args:
            joint_path (JointPath): Another JointPath object to prepend.

        Raises:
            ValueError: If the joint orders of the two JointPath objects are
                        not the same.
        """
        if self.joint_order != joint_path.joint_order:
            raise ValueError(
                "Joint orders must match to prepend JointPath objects."
            )

        self.joint_values = np.append(joint_path.joint_values,
                                      self.joint_values, axis=1)
        self.tool_activations = np.append(joint_path.tool_activations,
                                          self.tool_activations)

    def get_joint_configuration(self, index):
        """Returns the joint configuration at a specific index in the path.

        Args:
            index (int): The index of the joint configuration to retrieve.

        Returns:
            dict: A dictionary with joint names as keys and joint values as
                  values.
        """
        joint_configuration = {
            joint_name: self.joint_values[j, index]
            for j, joint_name in enumerate(self.joint_order)
        }
        return joint_configuration

    def __len__(self):
        """Returns the number of poses in the joint path."""
        return self.joint_values.shape[1]

    def __iter__(self):
        """Initializes the iterator for the JointPath."""
        self.current_index = 0
        return self

    def __next__(self):
        """Returns the next joint configuration and tool activation."""
        if self.current_index < len(self):
            i = self.current_index
            self.current_index += 1
            joint_configuration = self.get_joint_configuration(i)
            return joint_configuration, self.tool_activations[i]
        else:
            raise StopIteration
