from ompl import base as ob
from ompl import geometric as og
from collision_checker import CollisionChecker
from robot_base import RobotBase

INTERPOLATE_NUM = 500  # Number of segments for interpolating the solution path
DEFAULT_PLANNING_TIME = 5.0  # Default maximum allowed time for planning


class PathPlanner:
    def __init__(self, robot: RobotBase, collision_checker: CollisionChecker, planner_name="RRT", selected_joints: set = None):
        """
        Initializes the motion planning setup for a given robot in a constrained environment.

        Args:
            robot: Instance of RobotBase representing the robot model and its kinematics.
            collision_checker: An instance of CollisionChecker for managing collision detection.
        """
        self.robot = robot
        self.collision_checker = collision_checker

        if selected_joints is None:
            lower_limit, upper_limit = robot.get_joint_limits()
            self.selected_joints = robot.joint_name_to_index.keys()
        else:
            lower_limit, upper_limit = robot.get_joint_limits(selected_joints)
            self.selected_joints = selected_joints
        # Set bounds for the state space based on the robot's joint limits
        self.dimensions = len(lower_limit)
        bounds = ob.RealVectorBounds(self.dimensions)

        for i, (lower, upper) in enumerate(zip(lower_limit.values(), upper_limit.values())):
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)
        # Define the state space with dimensionality matching the robot's degrees of freedom
        self.space = ob.RealVectorStateSpace(self.dimensions)
        self.space.setBounds(bounds)

        # Initialize the motion planning problem setup
        self.ss = og.SimpleSetup(self.space)

        # Define a state validity checker to ensure collision-free states
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.is_state_valid)
        )

        # Retrieve the space information object for further configuration
        self.si = self.ss.getSpaceInformation()

        # Set the default planner
        self.set_planner(planner_name)

    def is_state_valid(self, state):
        """
        Checks if a given state is valid by ensuring no collisions with the environment
        or self-collisions.

        Args:
            state: The state of the robot (list of joint positions).

        Returns:
            bool: True if the state is valid (collision-free), False otherwise.
        """
        target = {}
        joint_positions = [state[i] for i in range(self.dimensions)]
        for joint_name, joint_position  in zip(self.selected_joints, joint_positions):
            target[joint_name] = joint_position
        self.robot.reset_joint_position(target)
        return self.collision_checker.check_collision()

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

    def plan_start_goal(self, start: dict, goal: dict, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plans a path between a start and goal state within the allowed time.

        Args:
            start: List of joint values representing the starting configuration.
            goal: List of joint values representing the goal configuration.
            allowed_time: Time allowed for the planner to find a solution.

        Returns:
            Tuple (bool, list): Success flag and the interpolated solution path.
        """
        orig_robot_state = start

        # Set start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i, (start_joint_value, goal_joint_value) in enumerate(zip(start.values(), goal.values())):
            s[i] = start_joint_value
            g[i] = goal_joint_value
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
        self.robot.reset_joint_position(orig_robot_state)
        return res, sol_path_list

    def state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.

        Args:
            state: An OMPL state object.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i in range(self.dimensions)]
