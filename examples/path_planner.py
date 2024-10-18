import pybullet_industrial as pi


class PathPlanner:
    def __init__(self, robot: pi.RobotBase, step_size, max_iterations):
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations

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
