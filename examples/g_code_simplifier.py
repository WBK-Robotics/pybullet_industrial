import numpy as np
import math
import matplotlib.pyplot as plt


class GCodeSimplifier:

    def __init__(self, input_g_code: list = None,
                 g_code_type: chr = 'cartesian'):
        self.g_code = input_g_code
        self.g_code_type = g_code_type
        self.input_points = None
        self.input_orientations = None

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
                    prev_point = np.array([line[key]
                                           for key in cartesian_order[:3]])
                    prev_orientation = np.array(
                        [line[key] for key in cartesian_order[3:]])
                    points.append(prev_point)
                    orientations.append(prev_orientation)
                elif prev_point is not None:  # If there's a previous point
                    # Use the previous point if current line doesn't have X, Y, or Z
                    point = np.array([line.get(key, prev_point[i])
                                      for i, key in enumerate(cartesian_order[:3])])
                    orientation = np.array(
                        [line.get(key, prev_orientation[i]) for i, key in enumerate(cartesian_order[3:])])
                    points.append(point)
                    orientations.append(orientation)

            elif self.g_code_type == 'joint_positions':
                if all(key in line for key in joint_order):
                    joint_positions.append(
                        np.array([line[key] for key in joint_order]))
                else:
                    # Handle cases where some joint positions may be missing
                    if joint_positions:
                        # If there are previously recorded joint positions, use them
                        last_joint_position = joint_positions[-1]
                        joint_position = np.array(
                            [line.get(key, last_joint_position[i]) for i, key in enumerate(joint_order)])
                        joint_positions.append(joint_position)

        # Convert lists to numpy arrays
        if self.g_code_type == 'cartesian':
            self.input_points = np.array(points)
            self.input_orientations = np.array(orientations)
        elif self.g_code_type == 'joint_positions':
            self.input_joint_positions = np.array(joint_positions)

    def simplify_g_code(self, epsilon):
        self.g_code_to_arrays()

        if self.g_code_type == 'cartesian':
            self.simplified_vector, self.simplified_indexes = self.simplify_vectors(
                np.concatenate((self.input_points, self.input_orientations), axis=1), epsilon)
            self.simplified_points, self.simplified_orientations = np.split(
                self.simplified_vector, 2, axis=1)

        elif self.g_code_type == 'joint_positions':
            self.simplified_joint_positions, self.simplified_indexes = self.simplify_vectors(
                self.input_joint_positions, epsilon)

        self.build_simpflified_g_code()

    def build_simpflified_g_code(self):
        self.g_code = []

        if self.g_code_type == 'cartesian':
            for i in self.simplified_vector:
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

    def simplify_vectors(self, vectors, epsilon, start_index=0):
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

        simplified_points = []
        simplified_indexes = []
        if max_distance > epsilon:
            # Recursively simplify the segments
            first_half, first_half_indexes = self.simplify_vectors(
                vectors[:max_index+1], epsilon, start_index)
            second_half, second_half_indexes = self.simplify_vectors(
                vectors[max_index:], epsilon, start_index + max_index)

            # Convert lists to numpy arrays
            first_half = np.array(first_half)
            second_half = np.array(second_half)

            # Concatenate numpy arrays
            simplified_points = np.concatenate(
                (first_half[:-1], second_half), axis=0)
            simplified_indexes = first_half_indexes[:-1] + second_half_indexes
        else:
            simplified_points = np.array([start_point, end_point])
            simplified_indexes = [start_index, start_index + len(vectors) - 1]

        return simplified_points, simplified_indexes

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

    def round_cartesian(self, round_xyz: int = 4, round_abc: int = 4):
        for g_code_line in self.g_code:
            for key in g_code_line:
                if key in ['X', 'Y', 'Z']:
                    g_code_line[key] = round(g_code_line[key], round_xyz)
                elif key in ['A', 'B', 'C']:
                    g_code_line[key] = round(g_code_line[key], round_abc)

    def round_joint_positions(self, round_dec: int = 4):
        search_keys = ['RA1', 'RA2', 'RA3', 'RA4', 'RA5', 'RA6']
        for g_code_line in self.g_code:
            for key in g_code_line:
                if key in search_keys:
                    g_code_line[key] = round(g_code_line[key], round_dec)

    def scale_g_code(self, scaling, keys_xyz):
        for g_code_line in self.g_code:
            for key in keys_xyz:
                if key in g_code_line:
                    g_code_line[key] *= scaling

    def add_offset_to_g_code(self, offset_dict, keys_xyz):
        for g_code_line in self.g_code:
            for key in keys_xyz:
                if key in g_code_line:
                    g_code_line[key] -= offset_dict[key]

    def convert_to_degrees(self, round_dec, keys_abc, degrees):
        for g_code_line in self.g_code:
            for key in keys_abc:
                if key in g_code_line:
                    if degrees:
                        g_code_line[key] = round(
                            math.degrees(g_code_line[key]), round_dec)
                    else:
                        g_code_line[key] = round(g_code_line[key], round_dec)

    def apply_feedrate(self, feedrate):
        for g_code_line in self.g_code:
            if 'G' in g_code_line and g_code_line['G'] == 1 and feedrate is not None:
                g_code_line['F'] = feedrate

    def skip_command(self, skip):
        self.g_code = [line for line in self.g_code if not any(
            key in line for key in skip)]
