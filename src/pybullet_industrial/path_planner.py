from ompl import base as ob
from ompl import geometric as og
from pybullet_industrial import CollisionChecker
from pybullet_industrial import RobotBase
from pybullet_industrial import JointPath
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
                 planner_name="RRT", selected_joint_names: set = None):
        self.robot = robot
        self.collision_checker = collision_checker

        self.joint_order = robot.get_moveable_joints(selected_joint_names)[0]
        self.real_vecotr = True
        if any(value == np.inf for value in robot.get_joint_limits(self.joint_order)[1].values()):
            self.real_vecotr = False
            self.space = self.build_compound_space()
        else:
            self.space = self.build_realvecotr_space()

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

    def build_realvecotr_space(self):
        # Configure bounds for the state space
        number_of_dimensions = len(self.joint_order)
        bounds = ob.RealVectorBounds(number_of_dimensions)
        lower_limit, upper_limit = self.robot.get_joint_limits(set(self.joint_order))

        for i, joint_name in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint_name])
            bounds.setHigh(i, upper_limit[joint_name])

        # Define the state space with dimensions matching the joint order
        space = ob.RealVectorStateSpace(number_of_dimensions)
        space.setBounds(bounds)
        return space

    def build_compound_space(self):
        """
        Builds the appropriate state space based on the robot's joint limits and types.

        Args:
            robot: Instance of RobotBase representing the robot model and its kinematics.
            joint_order: List of joint names to include in the planning space.

        Returns:
            ob.CompoundStateSpace: The constructed state space for planning.
        """
        # Retrieve joint limits and joint types from the robot
        lower_limit, upper_limit = self.robot.get_joint_limits(set(self.joint_order))

        # Define a compound state space to handle different joint types
        space = ob.CompoundStateSpace()

        for joint_name in self.joint_order:
            if lower_limit[joint_name] == -np.inf and upper_limit[joint_name] == np.inf:
                space.addSubspace(ob.SO2StateSpace(), 1.0)  # Continuous joint
            else:
                subspace = ob.RealVectorStateSpace(1)
                subspace.setBounds(lower_limit[joint_name], upper_limit[joint_name])
                space.addSubspace(subspace, 1.0)  # Revolute joint with limits

        return space

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
        if not self.real_vecotr:
            joint_positions = self.coumpound_state_to_list(state)
        else:
            joint_positions = self.realvector_state_to_list(state)

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
        s, g = self.set_start_goal_states(start, goal)
        self.ss.setStartAndGoalStates(s, g)

        # Solve the planning problem
        solved = self.ss.solve(allowed_time)
        res = False
        joint_path = None
        if solved:
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            if self.real_vecotr:
                sol_path_list = np.array(
                [self.realvector_state_to_list(state) for state in sol_path_states])
            else:
                sol_path_list = np.array(
                    [self.coumpound_state_to_list(state) for state in sol_path_states])
            joint_path = JointPath(sol_path_list.transpose(), self.joint_order)
            res = True
        else:
            print("No solution found")

        # Restore original robot state
        self.robot.reset_joint_position(orig_robot_state)
        return res, joint_path

    def set_start_goal_states(self, start, goal):
        # Set start and goal states
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i, joint_name in enumerate(self.joint_order):
            s[i] = start[joint_name]
            g[i] = goal[joint_name]
        return s, g

    def coumpound_state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.

        Args:
            state: An OMPL state object.

        Returns:
            list: A list of joint values.
        """
        joint_values = []
        for i in range(self.space.getSubspaceCount()):
            subspace = self.space.getSubspace(i)
            if isinstance(subspace, ob.SO2StateSpace):
                joint_values.append(state[i].value)
            elif isinstance(subspace, ob.RealVectorStateSpace):
                joint_values.append(state[i][0])
        return joint_values

    def realvector_state_to_list(self, state):
        """
        Converts an OMPL state object to a list of joint values.

        Args:
            state: An OMPL state object.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]
