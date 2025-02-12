import sys
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import (
    CollisionChecker, RobotBase, JointPath
)
import numpy as np
import pybullet as p

# Global constant for continuous joint bounds.
CONTINUOUS_JOINT_BOUNDS = 3 * np.pi

# Number of segments to interpolate along the planned path.
INTERPOLATE_NUM = 500
# Maximum allowed planning time in seconds.
DEFAULT_PLANNING_TIME = 5.0


class SamplingSpace:
    """
    Creates an OMPL state space based on the robot's joint limits and
    provides utilities for converting states between OMPL and the
    robot's joint configuration.

    This class always creates a RealVectorStateSpace. For each joint:
      - If a joint limit is infinite, the joint is treated as continuous and
        its bounds are set to [-CONTINUOUS_JOINT_BOUNDS,
        CONTINUOUS_JOINT_BOUNDS].
      - Otherwise, a small margin (±0.1) is added to the provided limits.

    Additionally, helper methods are provided to create start and goal states.

    Args:
        robot (RobotBase): The robot model including kinematics and joint
            information.
    """
    def __init__(self, robot: RobotBase):
        # Retrieve the order of moveable joints from the robot.
        self.joint_order = robot.get_moveable_joints()[0]
        # Obtain lower and upper joint limits as dictionaries.
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order)
        )
        self.robot = robot

        # Flag indicating the use of a RealVector state space.
        self.real_vector = True
        # Build the RealVector state space using the joint limits.
        self.space = self.build_realvector_space(lower_limit, upper_limit)

    def build_realvector_space(self, lower_limit: dict, upper_limit: dict):
        """
        Constructs a RealVectorStateSpace with appropriate bounds for each
        joint.

        For each joint:
          - If either bound is infinite, it is considered a continuous joint
            and the bounds are set to [-CONTINUOUS_JOINT_BOUNDS,
            CONTINUOUS_JOINT_BOUNDS].
          - Otherwise, the provided limits are expanded slightly (±0.1) for
            robustness.

        Args:
            lower_limit (dict): Lower limits for each joint.
            upper_limit (dict): Upper limits for each joint.

        Returns:
            ob.RealVectorStateSpace: The configured state space.
        """
        num_dims = len(self.joint_order)
        bounds = ob.RealVectorBounds(num_dims)
        # Set bounds for each joint based on its limits.
        for i, joint in enumerate(self.joint_order):
            if np.isinf(lower_limit[joint]) or \
               np.isinf(upper_limit[joint]):
                bounds.setLow(i, -CONTINUOUS_JOINT_BOUNDS)
                bounds.setHigh(i, CONTINUOUS_JOINT_BOUNDS)
            else:
                bounds.setLow(i, lower_limit[joint] - 0.1)
                bounds.setHigh(i, upper_limit[joint] + 0.1)
        space = ob.RealVectorStateSpace(num_dims)
        space.setBounds(bounds)
        return space

    def realvector_state_to_list(self, state: ob.State):
        """
        Converts an OMPL RealVector state into a list of joint values.

        Args:
            state (ob.State): An OMPL state object.

        Returns:
            list: The joint values extracted from the state.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]

    def get_space(self):
        """
        Returns the constructed OMPL state space.

        Returns:
            ob.StateSpace: The state space used for planning.
        """
        return self.space

    def set_state(self, state: ob.State):
        """
        Updates the robot's joint configuration to match the given OMPL state.

        This method sets the robot's joints to the values specified in the
        state. Note: Collision and constraint checking are not performed here.

        Args:
            state (ob.State): The state containing joint values.
        """
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        self.robot.reset_joint_position(
            dict(zip(self.joint_order, joint_positions)), True
        )

    def set_start_goal_states(self, start: dict, goal: dict):
        """
        Creates and returns the start and goal states for the planning
        problem.

        Args:
            start (dict): Dict with joint values for the start configuration.
            goal (dict): Dict with joint values for the goal configuration.

        Returns:
            tuple(ob.State, ob.State): The start and goal states.
        """
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i, joint in enumerate(self.joint_order):
            s[i] = start[joint]
            g[i] = goal[joint]
        return s, g


class ClearanceObjective(ob.StateCostIntegralObjective):
    """
    An optimization objective that encourages paths with high clearance from
    obstacles.

    The cost for a state is calculated as the reciprocal of the sum of extra
    cost contributions (plus a small constant to avoid division by zero),
    thereby favoring states that are farther from obstacles. Additional state
    cost functions can be included if provided.

    Args:
        si (ob.SpaceInformation): The OMPL space information.
        sampling_space (SamplingSpace): Instance for converting and applying
            states.
        state_cost_functions (iterable of callables, optional): Functions to
            compute additional state costs.
    """
    def __init__(self, si: ob.SpaceInformation, sampling_space: SamplingSpace,
                 state_cost_functions=None):
        super(ClearanceObjective, self).__init__(si, True)
        self.sampling_space = sampling_space
        self.state_cost_functions = state_cost_functions

    def stateCost(self, state: ob.State):
        """
        Computes the cost of a state based on its clearance from obstacles.

        Args:
            state (ob.State): The state for cost evaluation.

        Returns:
            ob.Cost: The computed cost as the reciprocal of the total cost.
        """
        self.sampling_space.set_state(state)
        total_cost = 0
        if self.state_cost_functions:
            for state_cost in self.state_cost_functions:
                total_cost += state_cost()
        return ob.Cost(1 / (total_cost + sys.float_info.min))


class ValidityChecker(ob.StateValidityChecker):
    """
    Checks whether a state is valid by ensuring it is collision-free and
    satisfies all constraints.

    This validity checker first updates the robot's configuration to the given
    state, then:
      - Evaluates all user-defined constraint functions.
      - Checks for collisions using the provided collision checkers.

    Args:
        space_information (ob.SpaceInformation): The OMPL space info.
        sampling_space (SamplingSpace): Instance for state conversion.
        collision_checker_list (list): List of collision checker objects.
        constraint_functions (iterable of callables, optional): Functions to
            evaluate additional constraints.
    """
    def __init__(self, space_information: ob.SpaceInformation,
                 sampling_space: SamplingSpace,
                 collision_checker_list: list, constraint_functions=None):
        super(ValidityChecker, self).__init__(space_information)
        self.sampling_space = sampling_space
        self.collision_checker_list = collision_checker_list
        self.constraint_functions = constraint_functions

    def isValid(self, state: ob.State):
        """
        Determines whether the given state is valid.

        A state is valid if it satisfies all user-defined constraints and is
        collision-free.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if the state is valid; otherwise, False.
        """
        self.sampling_space.set_state(state)
        if self.constraint_functions is not None:
            for constraint in self.constraint_functions:
                if not constraint():
                    return False
        for collision_checker in self.collision_checker_list:
            if not collision_checker.check_collision():
                return False
        return True


class PathPlanner:
    """
    Plans a motion path for a robot in an environment with obstacles and
    constraints.

    This class sets up the state space, defines the planning problem (including
    the optimization objective), and computes a valid path between the
    specified start and goal joint configurations.

    Args:
        robot (RobotBase): The robot instance used for planning.
        collision_checker_list (list): List of collision checker objects.
        planner_name (str, optional): The planner algorithm to use (e.g.,
            "BITstar"). Defaults to "BITstar".
        objective (str, optional): The optimization objective (e.g.,
            "PathLength"). Defaults to "PathLength".
        constraint_functions (iterable of callables, optional): Functions to
            evaluate additional constraints.
        state_cost_functions (iterable of callables, optional): Functions to
            compute extra state costs.
    """
    def __init__(self, robot: RobotBase, collision_checker_list: list,
                 planner_name: str = "BITstar", objective: str = "PathLength",
                 constraint_functions=None, state_cost_functions=None):

        self.robot = robot
        self.collision_checker_list = collision_checker_list

        # Create a state space for the robot using SamplingSpace.
        self.sampling_space = SamplingSpace(robot)
        self.space = self.sampling_space.get_space()
        self.real_vector = self.sampling_space.real_vector

        # Set up OMPL space information and attach a validity checker.
        self.space_information = ob.SpaceInformation(self.space)
        self.validity_checker = ValidityChecker(
            self.space_information, self.sampling_space,
            collision_checker_list, constraint_functions
        )
        self.space_information.setStateValidityChecker(
            self.validity_checker
        )
        self.space_information.setup()

        # Define the planning problem and assign an optimization objective.
        self.problem_definition = ob.ProblemDefinition(
            self.space_information
        )
        self.problem_definition.setOptimizationObjective(
            self.allocateObjective(objective, state_cost_functions)
        )

        # Allocate the planner based on the provided planner name.
        self.planner = self.allocatePlanner(planner_name)

    def allocateObjective(self, objectiveType: str,
                          state_cost_functions=None):
        """
        Allocates an optimization objective based on the specified type.

        Supported objectives include:
          - "PathClearance": Encourages paths with high clearance.
          - "PathLength": Minimizes path length.
          - "ThresholdPathLength": Minimizes path length with a set cost
            threshold.
          - "WeightedLengthAndClearanceCombo": A weighted combination of path
            length and clearance.

        Args:
            objectiveType (str): The type of optimization objective.
            state_cost_functions (iterable of callables, optional): Functions for
                additional state cost evaluation.

        Returns:
            ob.OptimizationObjective: The configured optimization objective.
        """
        if objectiveType.lower() == "pathclearance":
            return ClearanceObjective(
                self.space_information, self.sampling_space,
                state_cost_functions
            )
        elif objectiveType.lower() == "pathlength":
            return ob.PathLengthOptimizationObjective(
                self.space_information
            )
        elif objectiveType.lower() == "thresholdpathlength":
            obj = ob.PathLengthOptimizationObjective(
                self.space_information
            )
            obj.setCostThreshold(ob.Cost(1.51))
            return obj
        elif objectiveType.lower() == \
             "weightedlengthandclearancecombo":
            length_obj = ob.PathLengthOptimizationObjective(
                self.space_information
            )
            clear_obj = ClearanceObjective(
                self.space_information, self.sampling_space,
                state_cost_functions
            )
            opt = ob.MultiOptimizationObjective(
                self.space_information
            )
            opt.addObjective(length_obj, 5.0)
            opt.addObjective(clear_obj, 1.0)
            return opt
        else:
            ou.OMPL_ERROR(
                "The specified optimization objective is not implemented."
            )

    def allocatePlanner(self, plannerType: str):
        """
        Allocates and returns a planner instance based on the specified planner
        type.

        Supported planners include:
          - "BFMTstar"
          - "BITstar"
          - "FMTstar"
          - "InformedRRTstar"
          - "PRMstar"
          - "RRTstar"
          - "SORRTstar"

        Args:
            plannerType (str): The name of the planner to use.

        Returns:
            og.Planner: The allocated planner instance.
        """
        if plannerType.lower() == "bfmtstar":
            return og.BFMT(self.space_information)
        elif plannerType.lower() == "bitstar":
            return og.BITstar(self.space_information)
        elif plannerType.lower() == "fmtstar":
            return og.FMT(self.space_information)
        elif plannerType.lower() == "informedrrtstar":
            return og.InformedRRTstar(self.space_information)
        elif plannerType.lower() == "prmstar":
            return og.PRMstar(self.space_information)
        elif plannerType.lower() == "rrtstar":
            return og.RRTstar(self.space_information)
        elif plannerType.lower() == "sorrtstar":
            return og.SORRTstar(self.space_information)
        else:
            ou.OMPL_ERROR(
                "The specified planner type is not implemented."
            )

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = DEFAULT_PLANNING_TIME):
        """
        Plans a path between the specified start and goal joint
        configurations.

        This method:
          - Clears any previous solutions.
          - Sets up the start and goal states.
          - Runs the planner for up to the allowed planning time.
          - If a solution is found, interpolates the path and converts it
            into a JointPath.

        Args:
            start (dict): Dict with joint values for the start configuration.
            goal (dict): Dict with joint values for the goal configuration.
            allowed_time (float, optional): Maximum planning time in seconds.
                Defaults to DEFAULT_PLANNING_TIME.

        Returns:
            tuple(bool, JointPath): A tuple with a success flag and the computed
                JointPath.
        """
        self.problem_definition.clearSolutionPaths()
        orig_state = start
        s, g = self.sampling_space.set_start_goal_states(start, goal)
        self.problem_definition.setStartAndGoalStates(s, g)

        self.planner.setProblemDefinition(self.problem_definition)
        self.planner.clear()
        self.planner.setup()

        solved = self.planner.solve(allowed_time)
        res = False
        joint_path = None

        if solved:
            sol_path = self.problem_definition.getSolutionPath()
            sol_path.interpolate(INTERPOLATE_NUM)
            states = sol_path.getStates()
            path_list = np.array([
                self.sampling_space.realvector_state_to_list(st)
                for st in states
            ])
            joint_path = JointPath(
                path_list.transpose(),
                tuple(self.robot.get_moveable_joints()[0])
            )
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_state, True)
        return res, joint_path
