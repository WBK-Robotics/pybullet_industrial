import numpy as np


class JointPath:
    def __init__(self, joint_values: np.array, joint_order: tuple,
                 tool_activations: np.array):
        self.joint_values = joint_values
        self.joint_order = joint_order
        self.tool_activations = tool_activations

    def __len__(self):
        return len(self.joint_values[0])

    def __iter__(self):
        self.current_index = 0
        return self

    def __next__(self):
        if self.current_index <= len(self)-1:
            i = self.current_index
            self.current_index += 1
            joint_target = {}
            for j, joint_name in enumerate(self.joint_order):
                joint_target[joint_name] = self.joint_values[j][i]
            return joint_target, self.tool_activations[i]
        else:
            raise StopIteration