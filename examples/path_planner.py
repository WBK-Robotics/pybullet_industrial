import os
import pybullet as p
import numpy as np
import pybullet_industrial as pi
from robot_base import RobotBase


class PathPlanner:
    def __init__(self, robot: RobotBase, step_size=None, max_iterations=None):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.ignored_internal_collisions = set()

    def get_joint_name_mapping(self, body_id):
        joint_name_mapping = {}
        num_joints = p.getNumJoints(body_id)
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(body_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            child_link = joint_info[12]
            parent_index = joint_info[16]
            joint_name_mapping[joint_index] = {
                'joint_name': joint_name,
                'parent_index': parent_index,
                'child_link': child_link
            }
        return joint_name_mapping

    def get_link_name_mapping(self, body_id):
        # Get base link information
        body_info = p.getBodyInfo(body_id)
        base_name = body_info[0].decode('utf-8')  # Decode the base name
        link_mapping = {-1: base_name}  # Initialize with base link

        # Get joint name mapping
        joint_name_mapping = self.get_joint_name_mapping(body_id)

        # Map child links from the joint_name_mapping
        for joint_index, joint_data in joint_name_mapping.items():
            child_link_name = joint_data['child_link'].decode('utf-8')
            link_mapping[joint_index] = child_link_name

        return link_mapping

    def get_internal_collisions(self, detailed=False):
        """
        Detects and reports collisions, categorizing them as ignored or unignored.
        If detailed is True, prints detailed collision information.
        """
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)

        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)
        joint_name_mapping = self.get_joint_name_mapping(self.robot.urdf)

        total_collisions = len(contact_points)
        unignored_collisions = []
        ignored_collisions = []

        for contact in contact_points:
            link_a_index = contact[3]
            link_b_index = contact[4]
            link_a_name = link_name_mapping.get(link_a_index, "Unknown")
            link_b_name = link_name_mapping.get(link_b_index, "Unknown")

            collision_pair = tuple(sorted((link_a_name, link_b_name)))

            if collision_pair in self.ignored_internal_collisions:
                ignored_collisions.append((collision_pair, link_a_index, link_b_index))
            else:
                unignored_collisions.append((collision_pair, link_a_index, link_b_index))

        if not contact_points:
            print("No collisions detected.")
        else:
            print(f"Total Collisions: {total_collisions}")
            print(f"Unignored Collisions: {len(unignored_collisions)}")
            print(f"Ignored Collisions: {len(ignored_collisions)}")

            if detailed:
                print("\nUnignored Collisions:")
                for i, (collision, link_a_index, link_b_index) in enumerate(unignored_collisions, 1):
                    joint_info = None
                    for joint_index, joint_data in joint_name_mapping.items():
                        if set(collision) == {joint_data['parent_index'], joint_data['child_link'].decode('utf-8')}:
                            joint_info = joint_data
                            break

                    if joint_info:
                        print(f"Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index}) "
                              f"(Joint: {joint_info['joint_name']}, Parent Index: {joint_info['parent_index']}, "
                              f"Child: {joint_info['child_link'].decode('utf-8')})")
                    else:
                        print(f"Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index})")

                print("\nIgnored Collisions:")
                for i, (collision, link_a_index, link_b_index) in enumerate(ignored_collisions, 1):
                    print(f"Ignored Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index})")

        return unignored_collisions, ignored_collisions

    def update_ignored_internal_collisions(self, collisions):
        """
        Updates the ignored_internal_collisions set with normalized collision pairs.

        Args:
            collisions (list of tuples): Each tuple contains a collision pair and their indices.
        """
        normalized_collisions = set()
        for collision in collisions:
            collision_pair = collision[0]  # Extract the pair of link names
            normalized_collision = tuple(sorted((str(collision_pair[0]), str(collision_pair[1]))))  # Normalize as strings
            normalized_collisions.add(normalized_collision)
        self.ignored_internal_collisions.update(normalized_collisions)


    def get_ignored_collisions(self):
        """
        Returns the currently ignored collision pairs.
        """
        return self.ignored_internal_collisions

    def reset_ignored_collisions(self):
        """
        Resets the ignored_internal_collisions set.
        """
        self.ignored_internal_collisions = set()

    def path_planner(self, start_config, goal_config):
        pass

    def is_collision_free(self, sample):
        pass
