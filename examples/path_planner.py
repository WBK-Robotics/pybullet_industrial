import pybullet_industrial as pi
import pybullet as p
import numpy as np
from robot_base import RobotBase

class PathPlanner:
    def __init__(self, robot: RobotBase, step_size=None, max_iterations=None):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.ignored_collisions = []

    def update_ignored_collisions(self, collisions):
        """
        Updates the self.ignored_collisions list based on the given collision report.

        :param collisions: List of tuples containing pairs of link names to ignore.
        """
        for link_pair in collisions:
            if link_pair not in self.ignored_collisions:
                self.ignored_collisions.append(link_pair)

    def collision_report(self, detailed=False):
        """
        Generates and prints a collision report, considering ignored collisions.

        :param detailed: If True, prints detailed collision information.
        """
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)

        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)
        unignored_collisions = []
        ignored_collisions = []

        if not contact_points:
            print("No collisions detected.")
            return

        for contact in contact_points:
            link_a_name = link_name_mapping.get(contact[3], "Unknown")
            link_b_name = link_name_mapping.get(contact[4], "Unknown")

            if (link_a_name, link_b_name) in self.ignored_collisions or (link_b_name, link_a_name) in self.ignored_collisions:
                ignored_collisions.append((link_a_name, link_b_name))
            else:
                unignored_collisions.append((link_a_name, link_b_name))

        print(f"Total Collisions: {len(contact_points)}")
        print(f"Unignored Collisions: {len(unignored_collisions)}")
        print(f"Ignored Collisions: {len(ignored_collisions)}")

        if detailed:
            if unignored_collisions:
                print("\nUnignored Collisions:")
                for i, (link_a_name, link_b_name) in enumerate(unignored_collisions, 1):
                    print(f"Collision {i}:")
                    print(f"  Link A: {link_a_name} - Link B: {link_b_name}")

            if ignored_collisions:
                print("\nIgnored Collisions:")
                for i, (link_a_name, link_b_name) in enumerate(ignored_collisions, 1):
                    print(f"Ignored Collision {i}:")
                    print(f"  Link A: {link_a_name} - Link B: {link_b_name}")

    def get_collisions(self):
        """
        Retrieves the list of current unignored collisions.

        :return: List of tuples containing link names involved in unignored collisions.
        """
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)

        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)
        unignored_collisions = []

        for contact in contact_points:
            link_a_name = link_name_mapping.get(contact[3], "Unknown")
            link_b_name = link_name_mapping.get(contact[4], "Unknown")

            if (link_a_name, link_b_name) not in self.ignored_collisions and (link_b_name, link_a_name) not in self.ignored_collisions:
                unignored_collisions.append((link_a_name, link_b_name))

        return unignored_collisions

    def get_ignored_collisions(self):
        """
        Retrieves the list of current ignored collisions.

        :return: List of tuples containing link names involved in ignored collisions.
        """
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)

        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)
        ignored_collisions = []

        for contact in contact_points:
            link_a_name = link_name_mapping.get(contact[3], "Unknown")
            link_b_name = link_name_mapping.get(contact[4], "Unknown")

            if (link_a_name, link_b_name) in self.ignored_collisions or (link_b_name, link_a_name) in self.ignored_collisions:
                ignored_collisions.append((link_a_name, link_b_name))

        return ignored_collisions

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

    def path_planner(self, start_config, goal_config):
        pass

    def is_collision_free(self, sample):
        pass
