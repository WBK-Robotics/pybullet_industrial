from ompl import base as ob
from ompl import geometric as og
from collision_checker import CollisionChecker

INTERPOLATE_NUM = 500  # Number of segments for interpolating the solution path
DEFAULT_PLANNING_TIME = 5.0  # Default maximum allowed time for planning

class PbOMPL:
    def __init__(self, robot, collision_checker: CollisionChecker):
        """
        Initializes the motion planning setup for a given robot in a constrained environment.

        Args:
            robot: Instance of RobotBase representing the robot model and its kinematics.
            collision_checker: An instance of CollisionChecker for managing collision detection.
        """
        self.robot = robot
        self.collision_checker = collision_checker

        # Define the state space with dimensionality matching the robot's degrees of freedom
        self.space = ob.RealVectorStateSpace(robot.dof)

        # Set bounds for the state space based on the robot's joint limits
        lower_limit, upper_limit = robot.get_joint_limits()
        bounds = ob.RealVectorBounds(robot.dof)

        for i, (lower, upper) in enumerate(zip(lower_limit, upper_limit)):
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)

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
        self.set_planner("RRT")

    def is_state_valid(self, state):
        """
        Checks if a given state is valid by ensuring no collisions with the environment
        or self-collisions.

        Args:
            state: The state of the robot (list of joint positions).

        Returns:
            bool: True if the state is valid (collision-free), False otherwise.
        """
        joint_positions = [state[i] for i in range(self.robot.dof)]
        return self.collision_checker.is_state_valid(joint_positions)

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
        orig_robot_state = self.robot.moving_joint_positions

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
        self.robot.reset_joint_positions(orig_robot_state)
        return res, sol_path_list

    def plan(self, goal, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plans a path to the goal state from the robot's current state.

        Args:
            goal: List of joint values representing the goal configuration.
            allowed_time: Time allowed for the planner to find a solution.
        """
        start = self.robot.moving_joint_positions
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.

        Args:
            state: An OMPL state object.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i in range(self.robot.dof)]
