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
        if contact_points:
            print("COLLISION DETECTED")
            print(len(contact_points))
            for i, contact in enumerate(contact_points):
                print(f"Contact {i}:")
                print(contact)
        else:
            print("NO COLLISION")
        print("end of code")
    
    def path_planner(self, start_config, goal_config):
        pass

    def is_collision_free(self, sample):
        pass

