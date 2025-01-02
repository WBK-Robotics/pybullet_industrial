from ompl import base as ob
from ompl import geometric as og
import pybullet as p
import utils
from itertools import product
from robot_base import RobotBase

INTERPOLATE_NUM = 500  # Number of segments for interpolating the solution path
DEFAULT_PLANNING_TIME = 5.0  # Default maximum allowed time for planning

class PbOMPL:
    def __init__(self, robot: RobotBase, obstacles: list = []) -> None:
        """
        Initializes the motion planning setup for a given robot in a constrained environment.

        Args:
            robot: Instance of RobotBase representing the robot model and its kinematics.
            obstacles: List of obstacle IDs present in the environment (optional).
        """
        self.robot = robot
        self.robot_id = robot.urdf
        self.obstacles = obstacles

        # Define the state space with dimensionality matching the robot's degrees of freedom
        self.space = ob.RealVectorStateSpace(robot.num_dim)

        # Set bounds for the state space based on the robot's joint limits
        bounds = ob.RealVectorBounds(robot.num_dim)
        joint_bounds = self.robot.get_joint_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)

        # Initialize the motion planning problem setup
        self.ss = og.SimpleSetup(self.space)

        # Define a state validity checker to ensure collision-free states
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.is_state_valid))

        # Retrieve the space information object for further configuration
        self.si = self.ss.getSpaceInformation()

        # Configure collision detection and default planner
        self.set_obstacles(obstacles)
        self.set_planner("RRT")  # Default planner is RRT

    def set_obstacles(self, obstacles):
        """
        Updates the list of obstacles and reconfigures collision detection.
        """
        self.obstacles = obstacles
        self.setup_collision_detection(self.robot, self.obstacles)

    def add_obstacles(self, obstacle_id):
        """
        Adds a new obstacle to the list of tracked obstacles.
        """
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        """
        Removes an obstacle from the list of tracked obstacles.
        """
        self.obstacles.remove(obstacle_id)

    def is_state_valid(self, state):
        """
        Checks if a given state is valid by ensuring no collisions with the environment
        or self-collisions.
        """
        self.robot.set_robot(self.state_to_list(state))

        # Check for self-collisions
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                return False

        # Check for collisions with the environment
        for body1, body2 in self.check_body_pairs:
            if utils.pairwise_collision(body1, body2):
                return False

        return True

    def get_collision_links(self, robot: RobotBase):
        """
        Returns a list of robot links that are currently in collision.
        """
        allowed_collision_links = []
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(robot.urdf, link1, robot.urdf, link2):
                allowed_collision_links.append((link1, link2))
        return allowed_collision_links

    def setup_collision_detection(self, robot: RobotBase, obstacles, self_collisions=True, allow_collision_links=[]):
        """
        Configures the collision detection settings, including self-collision and environment checks.
        """
        self.check_link_pairs = utils.get_self_link_pairs(robot.urdf, robot.joint_idx) if self_collisions else []
        self.check_link_pairs = [pair for pair in self.check_link_pairs if pair not in allow_collision_links]

        moving_links = frozenset(
            [item for item in utils.get_moving_links(robot.urdf, robot.joint_idx) if item not in allow_collision_links])
        moving_bodies = [(robot.urdf, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, obstacles))

    def set_planner(self, planner_name):
        """
        Sets the motion planner to be used for path planning.

        Supported planners: PRM, RRT, RRTConnect, RRTstar, EST, FMT, BITstar.
        """
        planner_map = {
            "PRM": og.PRM,
            "RRT": og.RRT,
            "RRTConnect": og.RRTConnect,
            "RRTstar": og.RRTstar,
            "EST": og.EST,
            "FMT": og.FMT,
            "BITstar": og.BITstar,
        }
        if planner_name in planner_map:
            self.planner = planner_map[planner_name](self.ss.getSpaceInformation())
            self.ss.setPlanner(self.planner)
        else:
            print(f"{planner_name} not recognized. Please add it first.")

    def plan_start_goal(self, start, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plans a path between a start and goal state within the allowed time.

        Args:
            start: List of joint values representing the starting configuration.
            goal: List of joint values representing the goal configuration.
            allowed_time: Time allowed for the planner to find a solution.

        Returns:
            Tuple (bool, list): Success flag and the interpolated solution path.
        """
        orig_robot_state = self.robot.get_cur_state()

        # Set start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]
        self.ss.setStartAndGoalStates(s, g)

        # Solve the planning problem
        solved = self.ss.solve(allowed_time)
        res = False
        sol_path_list = []
        if solved:
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            res = True
        else:
            print("No solution found")

        # Restore original robot state
        self.robot.set_robot(orig_robot_state)
        return res, sol_path_list

    def plan(self, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plans a path to the goal state from the robot's current state.

        Args:
            goal: List of joint values representing the goal configuration.
            allowed_time: Time allowed for the planner to find a solution.
        """
        start = self.robot.get_cur_state()
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.
        """
        return [state[i] for i in range(self.robot.num_dim)]
