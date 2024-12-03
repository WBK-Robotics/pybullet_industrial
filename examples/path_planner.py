import pybullet_industrial as pi
import pybullet as p
import numpy as np
from robot_base import RobotBase


class PathPlanner:
    def __init__(self, robot: RobotBase, step_size=None, max_iterations=None):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.ignored_internal_collisions = set()

    def get_link_name_mapping(self, body_id):
        """
        Retrieves a mapping of link indices to link names for a given body.
        """
        num_joints = p.getNumJoints(body_id)
        link_name_mapping = {-1: "base"}  # -1 refers to the base link in PyBullet

        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(body_id, joint_index)
            link_name = joint_info[12].decode('utf-8')  # Extract the link name
            link_name_mapping[joint_index] = link_name

        return link_name_mapping

    def get_internal_collisions(self, detailed=False):
        """
        Detects and reports collisions, categorizing them as ignored or unignored.
        If detailed is True, prints detailed collision information.
        """
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)

        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)

        total_collisions = len(contact_points)
        unignored_collisions = []
        ignored_collisions = []

        for contact in contact_points:
            link_a_name = link_name_mapping.get(contact[3], "Unknown")
            link_b_name = link_name_mapping.get(contact[4], "Unknown")

            collision_pair = tuple(sorted((link_a_name, link_b_name)))

            if collision_pair in self.ignored_internal_collisions:
                ignored_collisions.append(collision_pair)
            else:
                unignored_collisions.append(collision_pair)

        if not contact_points:
            print("No collisions detected.")
        else:
            print(f"Total Collisions: {total_collisions}")
            print(f"Unignored Collisions: {len(unignored_collisions)}")
            print(f"Ignored Collisions: {len(ignored_collisions)}")

            if detailed:
                print("\nUnignored Collisions:")
                for i, collision in enumerate(unignored_collisions, 1):
                    print(f"Collision {i}: {collision[0]} - {collision[1]}")

                print("\nIgnored Collisions:")
                for i, collision in enumerate(ignored_collisions, 1):
                    print(f"Ignored Collision {i}: {collision[0]} - {collision[1]}")

        return unignored_collisions, ignored_collisions

    def update_ignored_internal_collisions(self, collisions):
        """
        Updates the ignored_internal_collisions set with new collision pairs.
        Ensures reverse duplicates are normalized and not added.
        """
        for collision in collisions:
            normalized_collision = tuple(sorted(collision))
            self.ignored_internal_collisions.add(normalized_collision)

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
