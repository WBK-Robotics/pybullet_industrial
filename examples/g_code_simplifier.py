import numpy as np
import copy
import math


class GCodeSimplifier:

    def __init__(self, raw_g_code: list = None):
        self.g_code = raw_g_code

    def simplify(self, epsilon):
        # Convert G-code to a list of robot positions for simplification
        points = []
        orientations = []
        prev_point = None
        order = ['X', 'Y', 'Z', 'A', 'B', 'C']
        for line in self.g_code:
            if all(key in line for key in order):
                prev_point = np.array([line[key] for key in order[:3]])
                prev_orientation = np.array([line[key] for key in order[3:]])
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

        simplified_points, simplified_points_indexes = self.simplify_points(
            points, epsilon)
        simplified_orientations, simpflified_orientations_indexes = self.simplify_points(
            orientations, epsilon)
        # Rebuild simplified G-code
        simplified_g_code = []
        for i, point in enumerate(simplified_points):
            # Assuming the index aligns, which may need adjustment
            line = self.g_code[i]
            simplified_g_code.append({
                'G': 1,
                'X': point[0],
                'Y': point[1],
                'Z': point[2],
                **{key: line[key] for key in line if key not in ['X', 'Y', 'Z']}
            })

        return simplified_g_code

    def simplify_points(self, points, epsilon, start_index=0):
        if len(points) < 3:
            return points, list(range(start_index, start_index + len(points)))

        start_point = points[0]
        end_point = points[-1]
        max_distance = 0
        max_index = 0
        for i in range(1, len(points) - 1):
            distance = self.distance_point_to_line(
                points[i], start_point, end_point)
            if distance > max_distance:
                max_distance = distance
                max_index = i

        simplified_points = []
        simplified_indexes = []
        if max_distance > epsilon:
            # Recursively simplify the segments
            first_half, first_half_indexes = self.simplify_points(
                points[:max_index+1], epsilon, start_index)
            second_half, second_half_indexes = self.simplify_points(
                points[max_index:], epsilon, start_index + max_index)

            simplified_points = first_half[:-1] + second_half
            simplified_indexes = first_half_indexes[:-1] + second_half_indexes
        else:
            simplified_points = [start_point, end_point]
            simplified_indexes = [start_index, start_index + len(points) - 1]

        return simplified_points, simplified_indexes

    def distance_point_to_line(self, point, start, end):
        # Calculate the distance of a point to a line segment
        if np.array_equal(start, end):
            return np.linalg.norm(point - start)
        return np.linalg.norm(np.cross(end-start, start-point)) / np.linalg.norm(end-start)

    def simplify_orientations(self):
        print("needs to be implement")

    def simplifiy_curves(self):
        print("nees to be implemented")

    def simpflify_joint_positions(self):
        print("needs to be implemented")

    # This needs to be tested

    def transform_g_code(self, g_code: list,
                         scaling: int = 1,
                         offset: list = [0, 0, 0],
                         round_dec: int = 4,
                         feedrate: int = None,
                         skip: list = ['M']):
        """
        Transform G-code lines based on specified parameters.

        Args:
            g_code (list): List of dictionaries representing G-code lines.
            scaling (int, optional): Scaling factor for coordinates. Defaults to 1.
            offset (list, optional): Offset values for X, Y, and Z axes. Defaults to [0, 0, 0].
            round_dec (int, optional): Number of decimal places for rounding. Defaults to 4.
            feedrate (int, optional): Feedrate value. Defaults to None.
            skip (list, optional): List of keys to skip transformation. Defaults to ['M'].

        Returns:
            list: Transformed G-code lines.
        """
        offset_dict = {'X': offset[0], 'Y': offset[1], 'Z': offset[2]}
        keys_xyz = {'X', 'Y', 'Z'}
        keys_abc = {'A', 'B', 'C'}

        modified_g_code = []

        for initial_line in g_code:
            should_skip = any(key in initial_line for key in skip)

            if not should_skip:
                modified_line = copy.deepcopy(initial_line)

                for key, value in initial_line.items():
                    if key in keys_xyz:
                        modified_line[key] = round(
                            (value - offset_dict[key]) * scaling, round_dec)

                    if key in keys_abc:
                        modified_line[key] = round(
                            math.degrees(value), round_dec)

                if 'G' in initial_line and initial_line['G'] == 1 and feedrate is not None:
                    modified_line['F'] = feedrate

                modified_g_code.append(modified_line)

        return modified_g_code
