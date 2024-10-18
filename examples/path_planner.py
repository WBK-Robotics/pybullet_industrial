import pybullet_industrial as pi
import pybullet as p
import numpy as np


class PathPlanner:
    def __init__(self, robot: pi.RobotBase, step_size=None,
                 max_iterations=None):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.number_of_joints = p.getNumJoints(robot.urdf)

        self.lower_joint_limit = np.zeros(6)
        self.upper_joint_limit = np.zeros(6)

        index = 0
        for joint_number in range(self.number_of_joints):
            joint_type = p.getJointInfo(robot.urdf, joint_number)[2]

            if joint_type == 0:
                lower_limit = p.getJointInfo(robot.urdf, joint_number)[8]
                upper_limit = p.getJointInfo(robot.urdf, joint_number)[9]
                self.lower_joint_limit[index] = lower_limit
                self.upper_joint_limit[index] = upper_limit
                index += 1
        print("s")

    def create_random_sample(self):
        pass

    def is_collision_free(self, sample):
        pass

    def find_nearest_sample(self):
        pass

    def local_planner(self):
        pass

    def plan_path(self):
        pass
