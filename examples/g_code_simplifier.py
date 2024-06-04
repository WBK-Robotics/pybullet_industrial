import numpy as np
import copy
import math


class GCodeSimplifier:

    def __init__(self, input_g_code: list = None):
        self.g_code = input_g_code
        self.input_points = None
        self.input_orientations = None

    def g_code_to_arrays(self):
        points = []
        orientations = []
        prev_point = None
        order = ['X', 'Y', 'Z', 'A', 'B', 'C']
        for line in self.g_code:
            if all(key in line for key in order):
                prev_point = np.array([line[key] for key in order[:3]])
                prev_orientation = np.array(
                    [line[key] for key in order[3:]])
                points.append(prev_point)
                orientations.append(prev_orientation)
            elif prev_point is not None:  # If there's a previous point
                # Use the previous point if current line doesn't have X, Y, or Z
                point = np.array([line[key] if key in line else prev_point[i]
                                  for i, key in enumerate(order[:3])])
                orientation = np.array(
                    [line[key] if key in line else prev_orientation[i] for i, key in enumerate(order[3:])])
                points.append(point)
                orientations.append(orientation)

        self.input_points = points
        self.input_orientations = orientations

    def simplify_cartesian(self, epsilon_points, epsilon_orientations):
        self.g_code_to_arrays()

        simppfliefied_points, simplified_points_indexes = self.simplify_vectors(
            self.input_points,
            epsilon_points)
        simppfliefied_orientations, simplified_orientations_indexes = self.simplify_vectors(
            self.input_orientations,
            epsilon_orientations)

        self.build_simpflified_g_code(
            simplified_points_indexes, simplified_orientations_indexes)

    def build_simpflified_g_code(self, points_indexes, orientations_indexes):
        simplified_index = list(
            set(points_indexes + orientations_indexes))
        simplified_index.sort()
        self.g_code = []

        for i in simplified_index:
            # Assuming the index aligns, which may need adjustment

            self.g_code.append({
                'G': 1,
                'X': self.input_points[i][0],
                'Y': self.input_points[i][1],
                'Z': self.input_points[i][2],
                'A': self.input_orientations[i][0],
                'B': self.input_orientations[i][1],
                'C': self.input_orientations[i][2],
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

            simplified_points = first_half[:-1] + second_half
            simplified_indexes = first_half_indexes[:-1] + second_half_indexes
        else:
            simplified_points = [start_point, end_point]
            simplified_indexes = [start_index, start_index + len(vectors) - 1]

        return simplified_points, simplified_indexes

    def distance_point_to_line(self, point, start, end):
        # Calculate the distance of a point to a line segment
        if np.array_equal(start, end):
            return np.linalg.norm(point - start)
        return np.linalg.norm(np.cross(end-start, start-point)) / np.linalg.norm(end-start)

    def simplifiy_curves(self):
        print("nees to be implemented")

    def simpflify_joint_positions(self):
        print("needs to be implemented")

    # This needs to be tested

    def round_g_code(self, round_dec):
        for g_code_line in self.g_code:
            for key in g_code_line:
                if isinstance(g_code_line[key], (int, float)):
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
