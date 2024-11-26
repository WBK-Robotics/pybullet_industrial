import pybullet_industrial as pi
import pybullet as p
import numpy as np
from robot_base import RobotBase


class PathPlanner:
    def __init__(self, robot: RobotBase, step_size=None,
                 max_iterations=None):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
 
    def collision_report(self):
        p.performCollisionDetection()
        contact_points = p.getContactPoints(self.robot.urdf)
        
        link_name_mapping = self.get_link_name_mapping(self.robot.urdf)
        # Print the mapping
        # for link_index, link_name in link_name_mapping.items():
        #     print(f"Link Index: {link_index}, Link Name: {link_name}")

        if not contact_points:
            print("No collisions detected.")
        else:
            print("Collision Report:")
            for i, contact in enumerate(contact_points, 1):
                link_a_name = link_name_mapping.get(contact[3], "Unknown")
                link_b_name = link_name_mapping.get(contact[4], "Unknown")
                print(f"Collision {i}:")
                print(f"  Link A (Body A): {link_a_name} - Link B (Body B): {link_b_name}")
                print(f"  Contact Position (World): {contact[5]}")
                print(f"  Normal on B (World): {contact[7]}")
                print(f"  Contact Distance: {contact[8]}")
                print(f"  Normal Force: {contact[9]}")
        print("end of code")


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

