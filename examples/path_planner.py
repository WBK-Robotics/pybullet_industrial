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
        self.number_of_joints = p.getNumJoints(robot.urdf)

        self.lower_joint_limit = np.zeros(6)
        self.upper_joint_limit = np.zeros(6)

        self.max_iterations

        index = 0
        for joint_number in range(self.number_of_joints):
            joint_type = p.getJointInfo(robot.urdf, joint_number)[2]

            if joint_type == 0:
                lower_limit = p.getJointInfo(robot.urdf, joint_number)[8]
                upper_limit = p.getJointInfo(robot.urdf, joint_number)[9]
                self.lower_joint_limit[index] = lower_limit
                self.upper_joint_limit[index] = upper_limit
                index += 1

    def path_planner(self, start_config, goal_config):

        total_distance = self.distance(start_config, goal_config)
        self.step_size = total_distance/100

        nodes = [start_config]

        for _ in range(self.max_iterations):
            # Sample Random Node in C-Space
            random_node = self.create_random_sample()

            # Find nearest node in the tree
            nearest_node = self.find_nearest_node(nodes, random_node)

            # Steer from the nearest node towards the random node
            new_node = self.steer(nearest_node, random_node)

            if self.is_collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                nodes.append(new_node)

    def distance(self, p1, p2):
        pass

    def steer(self):
        pass

    def create_random_sample(self):
        pass

    def is_collision_free(self, sample):
        pass

    def find_nearest_node(self):
        pass

    def local_planner(self):
        pass

    def plan_path(self):
        pass