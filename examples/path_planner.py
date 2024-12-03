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

    def get_link_name_mapping(self, body_id):
        """
        Retrieves a mapping of PyBullet unique body link indices to link names.

        Args:
            body_id (int): The ID of the robot's body in the simulation.

        Returns:
            Dict[int, str]: A mapping of link indices (PyBullet unique IDs) to link names.
        """
        # Create a reverse mapping to ensure correctness
        _link_name_to_index = {p.getBodyInfo(body_id)[0].decode('UTF-8'): -1}

        # Populate the mapping
        for joint_id in range(p.getNumJoints(body_id)):
            link_name = p.getJointInfo(body_id, joint_id)[12].decode('UTF-8')
            _link_name_to_index[link_name] = joint_id

        # Convert to the desired output format: {index: name}
        link_name_mapping = {index: name for name, index in _link_name_to_index.items()}

        return link_name_mapping



    def get_joint_name_mapping(self, body_id):
        """
        Retrieves a mapping of joint indices to joint names, types, and participating links.

        Args:
            body_id (int): The ID of the robot's body in the simulation.

        Returns:
            Dict[int, Tuple[str, int, str, str]]: A mapping of joint indices to a tuple containing:
                                                   - Joint name
                                                   - Joint type
                                                   - Parent link name
                                                   - Child link name
        """
        num_joints = p.getNumJoints(body_id)
        joint_name_mapping = {}

        link_name_mapping = self.get_link_name_mapping(body_id)

        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(body_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')  # Extract the joint name
            joint_type = joint_info[2]  # Joint type (e.g., revolute, prismatic)
            parent_link = link_name_mapping.get(joint_info[16], "Unknown")  # Parent link name
            child_link = joint_info[12].decode('utf-8')  # Child link name
            joint_name_mapping[joint_index] = (joint_name, joint_type, parent_link, child_link)

        return joint_name_mapping

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
                    for joint_index, (joint_name, joint_type, parent_link, child_link) in joint_name_mapping.items():
                        if set(collision) == {parent_link, child_link}:
                            joint_info = (joint_name, joint_type, parent_link, child_link)
                            break

                    if joint_info:
                        print(f"Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index}) "
                              f"(Joint: {joint_info[0]}, Type: {joint_info[1]}, Parent: {joint_info[2]}, Child: {joint_info[3]})")
                    else:
                        print(f"Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index})")

                print("\nIgnored Collisions:")
                for i, (collision, link_a_index, link_b_index) in enumerate(ignored_collisions, 1):
                    print(f"Ignored Collision {i}: {collision[0]} ({link_a_index}) - {collision[1]} ({link_b_index})")

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




