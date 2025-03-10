import copy
import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pybullet_industrial import GCodeProcessor, GCodeLogger
import pybullet as p
import pybullet_industrial as pi

def transform_eulers_in_gcode(g_code: list):
    return_code = copy.deepcopy(g_code)
    for command in return_code:
        if 'A' in command and 'B' in command and 'C' in command:
            euler_pb = np.array([command['A'], command['B'], command['C']])
            rot = R.from_euler('xyz', euler_pb)
            euler_siemens = rot.as_euler('XYZ')
            command['A'] = euler_siemens[0]
            command['B'] = euler_siemens[1]
            command['C'] = euler_siemens[2]
    return return_code

class GCodeSimplifier:

    def __init__(self, input_g_code: list = None, g_code_type: chr = None):
        self._g_code = input_g_code
        self._g_code_type = g_code_type
        self.input_points = None
        self.input_orientations = None
        self.simplified_joint_positions = None
        self.simplified_vectors = []
        self.simplified_indexes = []

    @property
    def g_code(self):
        return self._g_code

    @property
    def g_code_type(self):
        return self._g_code_type

    def set_g_code_and_type(self, g_code, g_code_type='cartesian'):
        """Sets g_code and g_code_type together to ensure they are synchronized."""
        if g_code is None or g_code_type is None:
            raise ValueError("Both g_code and g_code_type must be provided.")
        self._g_code = g_code
        self._g_code_type = g_code_type

    def plot_joint_positions(self):
        # Extract the number of samples for input data
        num_samples_input = len(self.input_joint_positions)

        joint_labels = ['RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']

        # Assign a color to each joint label
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']

        # Prepare data for plotting
        x_samples_input = np.arange(num_samples_input)

        # Plot for each joint position
        plt.figure(figsize=(12, 8))

        for i, (label, color) in enumerate(zip(joint_labels, colors)):
            # Extract joint positions for each joint
            input_joint_values = [pos[i] for pos in self.input_joint_positions]
            simplified_joint_values = [pos[i]
                                       for pos in self.simplified_joint_positions]

            # Plot input joint positions as points with assigned color
            plt.scatter(x_samples_input, input_joint_values,
                        label=f'{label} Input', marker='o', color=color, alpha=0.5)
            # Plot simplified joint positions as larger points with the same color
            plt.scatter(self.simplified_indexes, simplified_joint_values,
                        label=f'{label} Simplified', marker='^', color=color, s=100, edgecolor='black')

        # Plot vertical dotted lines at each simplified index
        for index in self.simplified_indexes:
            plt.axvline(x=index, color='gray', linestyle='--', linewidth=1)

        # Adding labels and title
        plt.xlabel('Sample Index')
        plt.ylabel('Joint Position')
        plt.title('Joint Positions Over Samples')
        plt.legend()

        # Show plot
        plt.show()

    def plot_orientations(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Extracting x, y, z coordinates from the input orientations
        x_input = [orientation[0] for orientation in self.input_orientations]
        y_input = [orientation[1] for orientation in self.input_orientations]
        z_input = [orientation[2] for orientation in self.input_orientations]

        # Extracting x, y, z coordinates from the simplified orientations
        x_simplified = [orientation[0]
                        for orientation in self.simplified_orientations]
        y_simplified = [orientation[1]
                        for orientation in self.simplified_orientations]
        z_simplified = [orientation[2]
                        for orientation in self.simplified_orientations]

        # Plotting the input orientations
        ax.scatter(x_input, y_input, z_input, c='r',
                   marker='o', label='Input Orientations', s=20)

        # Plotting the simplified orientations
        ax.scatter(x_simplified, y_simplified, z_simplified, c='b',
                   marker='^', label='Simplified Orientations', s=100)

        # Adding labels
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        # Adding a legend
        ax.legend()

        # Showing the plot
        plt.show()

    def plot_points(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Extracting x, y, z coordinates from the input points
        x_input = [point[0] for point in self.input_points]
        y_input = [point[1] for point in self.input_points]
        z_input = [point[2] for point in self.input_points]

        # Extracting x, y, z coordinates from the simplified points
        x_simplified = [point[0] for point in self.simplified_points]
        y_simplified = [point[1] for point in self.simplified_points]
        z_simplified = [point[2] for point in self.simplified_points]

        # Plotting the simplified points with larger marker size and different color
        ax.scatter(x_simplified, y_simplified, z_simplified,
                   c='b', marker='^', label='Simplified Points', s=100)

        # Plotting the input points
        ax.scatter(x_input, y_input, z_input, c='r',
                   marker='o', label='Input Points', alpha=0.5, s=20)
        # Adding labels
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        # Adding a legend
        ax.legend()

        # Showing the plot
        plt.show()

    def g_code_to_arrays(self):
        points = []
        orientations = []
        joint_positions = []
        prev_point = None
        prev_orientation = None

        # Define orders for Cartesian and Joint Position types
        cartesian_order = ['X', 'Y', 'Z', 'A', 'B', 'C']
        joint_order = ['RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']

        for line in self.g_code:
            if self.g_code_type == 'cartesian':
                if all(key in line for key in cartesian_order):
                    # Convert each value to float
                    prev_point = np.array([float(line[key])
                                          for key in cartesian_order[:3]])
                    prev_orientation = np.array(
                        [float(line[key]) for key in cartesian_order[3:]])
                    points.append(prev_point)
                    orientations.append(prev_orientation)
                elif prev_point is not None:  # If there's a previous point
                    # Use the previous point if current line doesn't have X, Y, or Z
                    point = np.array([float(line.get(key, prev_point[i]))
                                     for i, key in enumerate(cartesian_order[:3])])
                    orientation = np.array([float(line.get(
                        key, prev_orientation[i])) for i, key in enumerate(cartesian_order[3:])])
                    points.append(point)
                    orientations.append(orientation)

            elif self.g_code_type == 'joint_positions':
                if all(key in line for key in joint_order):
                    # Convert each value to float
                    joint_positions.append(
                        np.array([float(line[key]) for key in joint_order]))
                else:
                    # Handle cases where some joint positions may be missing
                    if joint_positions:
                        # If there are previously recorded joint positions, use them
                        last_joint_position = joint_positions[-1]
                        joint_position = np.array(
                            [float(line.get(key, last_joint_position[i])) for i, key in enumerate(joint_order)])
                        joint_positions.append(joint_position)

        # Convert lists to numpy arrays
        if self.g_code_type == 'cartesian':
            self.input_points = np.array(points)
            self.input_orientations = np.array(orientations)
        elif self.g_code_type == 'joint_positions':
            self.input_joint_positions = np.array(joint_positions)

    def simplify_g_code(self, epsilon, split_angle=0):
        self.g_code_to_arrays()

        if self.g_code_type == 'cartesian':
            self.input_vectors = np.concatenate(
                (self.input_points, self.input_orientations), axis=1)
        elif self.g_code_type == 'joint_positions':
            self.input_vectors = self.input_joint_positions

        _, edge_indexes = self.simplify_directions(
            self.input_vectors, split_angle)

        self.simplified_vectors = []
        self.simplified_indexes = []

        for i in range(len(edge_indexes) - 1):
            start_index = edge_indexes[i]
            end_index = edge_indexes[i + 1]

            points, relative_indexes = self.simplify_douglas_peuker(
                self.input_vectors[start_index:end_index + 1], epsilon)

            # Convert relative indexes to absolute indexes
            absolute_indexes = [
                index + start_index for index in relative_indexes]

            # Append all but the last point and index to avoid redundancy
            self.simplified_vectors.append(points[:-1])
            self.simplified_indexes.extend(absolute_indexes[:-1])

        # Add the last point and index to ensure it's included
        self.simplified_vectors.append(self.input_vectors[edge_indexes[-1]])
        self.simplified_indexes.append(edge_indexes[-1])

        # Convert list of arrays into a single array
        self.simplified_vectors = np.vstack(self.simplified_vectors)

        if self.g_code_type == 'cartesian':
            self.simplified_points, self.simplified_orientations = np.split(
                self.simplified_vectors, 2, axis=1)
        elif self.g_code_type == 'joint_positions':
            self.simplified_joint_positions = self.simplified_vectors

        self.build_simplified_g_code()

    def build_simplified_g_code(self):
        self.set_g_code_and_type([], self.g_code_type)
        if self.g_code_type == 'cartesian':
            for i in self.simplified_vectors:
                self.g_code.append({
                    'G': 1,
                    'X': i[0],
                    'Y': i[1],
                    'Z': i[2],
                    'A': i[3],
                    'B': i[4],
                    'C': i[5]
                })

        elif self.g_code_type == 'joint_positions':
            for i in self.simplified_joint_positions:
                self.g_code.append({
                    'G': 1,
                    'RA1': i[0],
                    'RA2': i[1],
                    'RA3': i[2],
                    'RA4': i[3],
                    'RA5': i[4],
                    'RA6': i[5]
                })

    def simplify_directions(self, vectors, split_angle):
        # Initialize the list of kept points with the first point
        edge_indexes = [0]  # Start with the first point
        edge_vectors = [vectors[0]]

        # Iterate through the vector list
        for i in range(1, len(vectors) - 1):
            # Calculate the direction from the previous kept point to the current point
            direction_prev = vectors[i] - vectors[edge_indexes[-1]]
            direction_next = vectors[i + 1] - vectors[i]

            # Normalize the direction vectors
            norm_prev = np.linalg.norm(direction_prev)
            norm_next = np.linalg.norm(direction_next)

            # If either norm is zero, skip this point
            if norm_prev == 0 or norm_next == 0:
                continue

            direction_prev /= norm_prev
            direction_next /= norm_next

            # Calculate the angle between the two direction vectors using the dot product
            cos_angle = np.dot(direction_prev, direction_next)

            # Calculate the angular change (1 - cos_angle gives the small-angle approximation)
            angle_change = cos_angle * -1

            # If the angle change is greater than epsilon, keep the current point
            if angle_change > split_angle:
                edge_indexes.append(i)
                edge_vectors.append(vectors[i])

        # Always keep the last point
        edge_indexes.append(len(vectors) - 1)
        edge_vectors.append(vectors[-1])

        # Convert the list of simplified vectors to a NumPy array
        edge_vectors = np.array(edge_vectors)

        return edge_vectors, edge_indexes

    def simplify_douglas_peuker(self, vectors, epsilon, start_index=0):
        if len(vectors) < 3:
            return vectors, list(range(start_index, start_index + len(vectors)))

        start_point = vectors[0]
        end_point = vectors[-1]
        max_distance = 0
        max_index = 0
        for i in range(1, len(vectors) - 1):
            distance = self.distance_point_to_line(
                vectors[i], start_point, end_point)

            if distance > max_distance:
                max_distance = distance
                max_index = i

        simplified_vectors = []
        simplified_indexes = []
        if max_distance > epsilon:
            # Recursively simplify the segments
            first_half, first_half_indexes = self.simplify_douglas_peuker(
                vectors[:max_index+1], epsilon, start_index)
            second_half, second_half_indexes = self.simplify_douglas_peuker(
                vectors[max_index:], epsilon, start_index + max_index)

            # Convert lists to numpy arrays
            first_half = np.array(first_half)
            second_half = np.array(second_half)

            # Concatenate numpy arrays
            simplified_vectors = np.concatenate(
                (first_half[:-1], second_half), axis=0)
            simplified_indexes = first_half_indexes[:-1] + second_half_indexes
        else:
            simplified_vectors = np.array([start_point, end_point])
            simplified_indexes = [start_index, start_index + len(vectors) - 1]

        return simplified_vectors, simplified_indexes

    def distance_point_to_line(self, point, start, end):
        # Convert inputs to numpy arrays
        point = np.array(point)
        start = np.array(start)
        end = np.array(end)

        # If start and end are the same, return the distance from point to this single point
        if np.array_equal(start, end):
            return np.linalg.norm(point - start)

        # Direction vector of the line segment
        direction = end - start

        # Vector from start to the point
        vector_from_start_to_point = point - start

        # Project vector_from_start_to_point onto direction
        projection_length = np.dot(
            vector_from_start_to_point, direction) / np.dot(direction, direction)
        projected_vector = projection_length * direction

        # Perpendicular component
        perpendicular_vector = vector_from_start_to_point - projected_vector

        # Perpendicular distance
        distance = np.linalg.norm(perpendicular_vector)

        return distance

    @ staticmethod
    def round_cartesian(g_code, round_xyz: int = 4, round_abc: int = 4):
        for g_code_line in g_code:
            for key in g_code_line:
                if key in ['X', 'Y', 'Z']:
                    g_code_line[key] = round(g_code_line[key], round_xyz)
                elif key in ['A', 'B', 'C']:
                    g_code_line[key] = round(g_code_line[key], round_abc)
        return g_code

    @ staticmethod
    def round_joint_positions(g_code, round_dec: int = 5):
        search_keys = ['RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']
        for command in g_code:
            for key in search_keys:
                if key in command:
                    command[key] = round(command[key], round_dec)
        return g_code

    @ staticmethod
    def scale_g_code(g_code, scaling, keys):
        for command in g_code:
            for key in keys:
                if key in command:
                    command[key] *= scaling
        return g_code

    @ staticmethod
    def add_offset_to_g_code(g_code, offset_dict):
        for command in g_code:
            for key in offset_dict:
                if key in command:
                    command[key] += offset_dict[key]
        return g_code

    @ staticmethod
    def convert_to_radians(g_code):
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith('RA'):
                    command[key] = np.radians(command[key])
        return g_code

    @ staticmethod
    def convert_to_degrees(g_code):
        for command in g_code:
            for key in command:
                if key in {'A', 'B', 'C'} or key.startswith('RA'):
                    command[key] = np.degrees(command[key])
        return g_code

    @ staticmethod
    def apply_feedrate(g_code, feedrate):
        for g_code_line in g_code:
            if 'G' in g_code_line and g_code_line['G'] == 1 and feedrate is not None:
                g_code_line['F'] = feedrate
        return g_code

    @ staticmethod
    def skip_command(g_code, skip_commands):
        filtered_g_code = []
        for command in g_code:
            # Remove the key-value pairs that match the commands to skip
            for key in list(skip_commands.keys()):
                if key in command and command[key] == skip_commands[key]:
                    del command[key]
            # Add to the filtered list only if the command is not empty
            if command:
                filtered_g_code.append(command)

        return filtered_g_code


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_robot: str = os.path.join(dirname, 'robot_descriptions',
                                   'comau_nj290_robot.urdf')
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([0, 0, 0])
    p.connect(p.GUI)
    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
    processor = GCodeProcessor(robot=robot)
    simplifier = GCodeSimplifier()

    textfile = os.path.join(dirname, 'g_codes',
                                   'joint_path_planner.txt')

    with open(textfile, encoding='utf-8') as f:
        gcode_input = f.read()

    processor.g_code = processor.read_g_code(gcode_input)
    processor.g_code = transform_eulers_in_gcode(processor.g_code)
    processor.g_code = GCodeSimplifier.add_offset_to_g_code(processor.g_code, {'X': -4.0, 'Y': 6.5, 'Z': 0.0})
    processor.g_code = GCodeSimplifier.scale_g_code(
        processor.g_code, 1000.0, ['X', 'Y', 'Z'])
    processor.g_code = GCodeSimplifier.convert_to_degrees(processor.g_code)
    processor.g_code = GCodeSimplifier.round_cartesian(processor.g_code, 4, 4)
    processor.g_code = GCodeSimplifier.apply_feedrate(processor.g_code, 5000)
    exportfile = os.path.join(dirname, 'g_codes',
                                   'transformed.mpf')
    pi.GCodeLogger.write_g_code(
        processor.g_code, exportfile, {},
        postfix="M30\n")