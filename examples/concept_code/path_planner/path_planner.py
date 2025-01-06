from ompl import base as ob
from ompl import geometric as og
from collision_checker import CollisionChecker
from robot_base import RobotBase
from joint_path import JointPath
import numpy as np

INTERPOLATE_NUM = 500  # Number of segments for interpolating the solution path
DEFAULT_PLANNING_TIME = 5.0  # Default maximum allowed time for planning


class PathPlanner:
    """
    A class for setting up and managing motion planning for a robot in a
    constrained environment.

    Args:
        robot: Instance of RobotBase representing the robot model and its
               kinematics.
        collision_checker: Instance of CollisionChecker for collision
                           detection management.
        planner_name: Name of the planner to use for path planning.
                      Defaults to "RRT".
        selected_joints: Set of joint names to include in the planning space.
                         Defaults to None (uses all robot joints).
    """

    def __init__(self, robot: RobotBase, collision_checker: CollisionChecker,
                 planner_name="RRT", selected_joints: set = None):
        self.robot = robot
        self.collision_checker = collision_checker

        # Determine the joint order
        if selected_joints is None:
            joint_items = self.robot.joint_name_to_index.items()
        else:
            joint_items = [
                (joint_name, self.robot.joint_name_to_index[joint_name])
                for joint_name in selected_joints
            ]

        # Sort the joints by their indices
        sorted_joint_items = sorted(joint_items, key=lambda item: item[1])

        # Extract the joint names in the correct order
        self.joint_order = tuple(key for key, _ in sorted_joint_items)

        # Retrieve joint limits from the robot
        lower_limit, upper_limit = self.robot.get_joint_limits(
            set(self.joint_order)
        )

        # Configure bounds for the state space
        number_of_dimensions = len(self.joint_order)
        bounds = ob.RealVectorBounds(number_of_dimensions)

        for i, joint_name in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint_name])
            bounds.setHigh(i, upper_limit[joint_name])

        # Define the state space with dimensions matching the joint order
        self.space = ob.RealVectorStateSpace(number_of_dimensions)
        self.space.setBounds(bounds)

        # Initialize the motion planning problem setup
        self.ss = og.SimpleSetup(self.space)

        # Set a state validity checker for collision-free validation
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.is_state_valid)
        )

        # Retrieve the space information object for further configuration
        self.si = self.ss.getSpaceInformation()

        # Set the default planner
        self.set_planner(planner_name)

    def is_state_valid(self, state):
        """
        Checks if a given state is valid by ensuring no collisions with the
        environment or self-collisions.

        Args:
            state: The state of the robot (list of joint positions).

        Returns:
            bool: True if the state is valid (collision-free), False otherwise.
        """
        target = {}
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        for joint_name, joint_position in zip(self.joint_order, joint_positions):
            target[joint_name] = joint_position
        self.robot.reset_joint_position(target)
        return self.collision_checker.check_collision()

    def set_planner(self, planner_name):
        """
        Sets the motion planner to be used for path planning.

        Supported planners: PRM, RRT, RRTConnect, RRTstar, EST, FMT, BITstar.

        Args:
            planner_name: The name of the planner to set.
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

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plans a path between a start and goal state within the allowed time.

        Args:
            start: Dictionary of joint values representing the starting
                   configuration.
            goal: Dictionary of joint values representing the goal
                  configuration.
            allowed_time: Maximum time allowed for the planner to find a
                          solution.

        Returns:
            Tuple[bool, JointPath]: Success flag and the interpolated solution
                                    path as a JointPath object.
        """
        orig_robot_state = start

        # Set start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i, joint_name in enumerate(self.joint_order):
            s[i] = start[joint_name]
            g[i] = goal[joint_name]
        self.ss.setStartAndGoalStates(s, g)

        # Solve the planning problem
        solved = self.ss.solve(allowed_time)
        res = False
        joint_path = None
        if solved:
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = np.array(
                [self.state_to_list(state) for state in sol_path_states]
            )
            joint_path = JointPath(sol_path_list.transpose(), self.joint_order)
            res = True
        else:
            print("No solution found")

        # Restore original robot state
        self.robot.reset_joint_position(orig_robot_state)
        return res, joint_path

    def state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.

        Args:
            state: An OMPL state object.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]
