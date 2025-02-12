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


class SamplingSpace(ob.SpaceInformation):
    """
    Inherits from ob.SpaceInformation and encapsulates the robot's state
    space and helper functions.

    This class builds a RealVectorStateSpace from the robot's joint
    limits and provides utilities to convert between OMPL states and
    the robot's joint configuration. For each joint:
      - If a joint limit is infinite, the joint is treated as
        continuous with bounds:
        [-CONTINUOUS_JOINT_BOUNDS, CONTINUOUS_JOINT_BOUNDS].
      - Otherwise, a small margin (±0.1) is added to the limits.

    Args:
        robot (RobotBase): The robot model with joint and kinematic info.
    """
    def __init__(self, robot: RobotBase):
        # Retrieve the order of moveable joints.
        self.joint_order = robot.get_moveable_joints()[0]
        # Get lower and upper joint limits.
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order)
        )
        self.robot = robot
        # Build the state space using joint limits.
        space = self.build_realvector_space(lower_limit, upper_limit)
        # Initialize the base class with the created state space.
        super().__init__(space)

    def build_realvector_space(self, lower_limit: dict,
                               upper_limit: dict):
        """
        Constructs a RealVectorStateSpace with appropriate bounds for
        each joint.

        For each joint:
          - If either limit is infinite, the bounds are set to
            [-CONTINUOUS_JOINT_BOUNDS, CONTINUOUS_JOINT_BOUNDS].
          - Otherwise, the limits are expanded by ±0.1.

        Args:
            lower_limit (dict): Lower limits for joints.
            upper_limit (dict): Upper limits for joints.

        Returns:
            ob.RealVectorStateSpace: The configured state space.
        """
        num_dims = len(self.joint_order)
        bounds = ob.RealVectorBounds(num_dims)
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
        Converts an OMPL RealVector state to a list of joint values.

        Args:
            state (ob.State): An OMPL state object.

        Returns:
            list: The joint values from the state.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]

    def set_state(self, state: ob.State):
        """
        Updates the robot's joint configuration to match the given state.

        Note: Collision and constraint checking are not performed here.

        Args:
            state (ob.State): The state with joint values.
        """
        joint_positions = [
            state[i] for i, _ in enumerate(self.joint_order)
        ]
        self.robot.reset_joint_position(
            dict(zip(self.joint_order, joint_positions)), True
        )

    def set_start_goal_states(self, start: dict, goal: dict):
        """
        Creates and returns start and goal states for planning.

        Args:
            start (dict): Joint values for the start configuration.
            goal (dict): Joint values for the goal configuration.

        Returns:
            tuple(ob.State, ob.State): The start and goal states.
        """
        s = ob.State(self.getStateSpace())
        g = ob.State(self.getStateSpace())
        for i, joint in enumerate(self.joint_order):
            s[i] = start[joint]
            g[i] = goal[joint]
        return s, g

    def get_space(self):
        """
        Returns the state space (a RealVectorStateSpace).

        Returns:
            ob.StateSpace: The state space used for planning.
        """
        return self.getStateSpace()


class ClearanceObjective(ob.StateCostIntegralObjective):
    """
    An optimization objective that encourages paths with high
    clearance from obstacles.

    The cost is the reciprocal of the sum of extra cost contributions
    (plus a small constant to avoid division by zero), favoring states
    farther from obstacles.

    Args:
        si (ob.SpaceInformation): The OMPL space information.
        sampling_space (SamplingSpace): Instance for state conversion.
        state_cost_functions (iterable of callables, optional): Functions
            to compute additional state costs.
    """
    def __init__(self, si: ob.SpaceInformation,
                 sampling_space: SamplingSpace,
                 state_cost_functions=None):
        super(ClearanceObjective, self).__init__(si, True)
        self.sampling_space = sampling_space
        self.state_cost_functions = state_cost_functions

    def stateCost(self, state: ob.State):
        """
        Computes the cost of a state based on its clearance.

        Args:
            state (ob.State): The state for cost evaluation.

        Returns:
            ob.Cost: The computed cost.
        """
        self.sampling_space.set_state(state)
        total_cost = 0
        if self.state_cost_functions:
            for state_cost in self.state_cost_functions:
                total_cost += state_cost()
        return ob.Cost(1 / (total_cost + sys.float_info.min))


class ValidityChecker(ob.StateValidityChecker):
    """
    Checks whether a state is valid by verifying it is collision-free and
    satisfies all constraints.

    This checker updates the robot's configuration to the state and then:
      - Evaluates user-defined constraint functions.
      - Checks collisions using provided collision checkers.

    Args:
        space_information (ob.SpaceInformation): The OMPL space info.
        sampling_space (SamplingSpace): Instance for state conversion.
        collision_checker_list (list): Collision checker objects.
        constraint_functions (iterable of callables, optional): Functions to
            evaluate additional constraints.
    """
    def __init__(self, space_information: ob.SpaceInformation,
                 sampling_space: SamplingSpace,
                 collision_checker_list: list,
                 constraint_functions=None):
        super(ValidityChecker, self).__init__(space_information)
        self.sampling_space = sampling_space
        self.collision_checker_list = collision_checker_list
        self.constraint_functions = constraint_functions

    def isValid(self, state: ob.State):
        """
        Determines if a state is valid.

        A state is valid if it satisfies all constraints and is
        collision-free.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if valid, False otherwise.
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
    Plans a motion path for a robot in an environment with obstacles
    and constraints.

    This class sets up the planning problem, including the state space,
    optimization objective, and planner, and computes a valid path between
    the start and goal joint configurations.

    Args:
        robot (RobotBase): The robot instance used for planning.
        collision_checker_list (list): Collision checker objects.
        planner_name (str, optional): The planner algorithm (e.g.,
            "BITstar"). Defaults to "BITstar".
        objective (str, optional): The optimization objective (e.g.,
            "PathLength"). Defaults to "PathLength".
        constraint_functions (iterable of callables, optional): Functions to
            evaluate extra constraints.
        state_cost_functions (iterable of callables, optional): Functions to
            compute extra state costs.
    """
    def __init__(self, robot: RobotBase,
                 collision_checker_list: list,
                 planner_name: str = "BITstar",
                 objective: str = "PathLength",
                 constraint_functions=None,
                 state_cost_functions=None):

        self.robot = robot
        self.collision_checker_list = collision_checker_list

        # Create a state space that also acts as space information.
        self.sampling_space = SamplingSpace(robot)

        # Set up and attach a validity checker.
        self.validity_checker = ValidityChecker(
            self.sampling_space, self.sampling_space,
            collision_checker_list, constraint_functions
        )
        self.sampling_space.setStateValidityChecker(
            self.validity_checker
        )
        self.sampling_space.setup()

        # Define the planning problem with an optimization objective.
        self.problem_definition = ob.ProblemDefinition(
            self.sampling_space
        )
        self.problem_definition.setOptimizationObjective(
            self.allocateObjective(objective, state_cost_functions)
        )

        # Allocate the planner based on the given planner name.
        self.planner = self.allocatePlanner(planner_name)

    def allocateObjective(self, objectiveType: str,
                          state_cost_functions=None):
        """
        Allocates an optimization objective based on the type.

        Supported objectives:
          - "PathClearance": Encourages high clearance.
          - "PathLength": Minimizes path length.
          - "ThresholdPathLength": Minimizes path length with a cost
            threshold.
          - "WeightedLengthAndClearanceCombo": Weighted combo of path length
            and clearance.

        Args:
            objectiveType (str): The type of optimization objective.
            state_cost_functions (iterable of callables, optional): Functions
                for additional state cost evaluation.

        Returns:
            ob.OptimizationObjective: The configured objective.
        """
        if objectiveType.lower() == "pathclearance":
            return ClearanceObjective(
                self.sampling_space, self.sampling_space,
                state_cost_functions
            )
        elif objectiveType.lower() == "pathlength":
            return ob.PathLengthOptimizationObjective(
                self.sampling_space
            )
        elif objectiveType.lower() == "thresholdpathlength":
            obj = ob.PathLengthOptimizationObjective(
                self.sampling_space
            )
            obj.setCostThreshold(ob.Cost(1.51))
            return obj
        elif objectiveType.lower() == \
             "weightedlengthandclearancecombo":
            length_obj = ob.PathLengthOptimizationObjective(
                self.sampling_space
            )
            clear_obj = ClearanceObjective(
                self.sampling_space, self.sampling_space,
                state_cost_functions
            )
            opt = ob.MultiOptimizationObjective(
                self.sampling_space
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
        Allocates and returns a planner based on the specified type.

        Supported planners:
          - "BFMTstar"
          - "BITstar"
          - "FMTstar"
          - "InformedRRTstar"
          - "PRMstar"
          - "RRTstar"
          - "SORRTstar"

        Args:
            plannerType (str): The name of the planner.

        Returns:
            og.Planner: The allocated planner instance.
        """
        if plannerType.lower() == "bfmtstar":
            return og.BFMT(self.sampling_space)
        elif plannerType.lower() == "bitstar":
            return og.BITstar(self.sampling_space)
        elif plannerType.lower() == "fmtstar":
            return og.FMT(self.sampling_space)
        elif plannerType.lower() == "informedrrtstar":
            return og.InformedRRTstar(self.sampling_space)
        elif plannerType.lower() == "prmstar":
            return og.PRMstar(self.sampling_space)
        elif plannerType.lower() == "rrtstar":
            return og.RRTstar(self.sampling_space)
        elif plannerType.lower() == "sorrtstar":
            return og.SORRTstar(self.sampling_space)
        else:
            ou.OMPL_ERROR(
                "The specified planner type is not implemented."
            )

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = DEFAULT_PLANNING_TIME):
        """
        Plans a path between the specified start and goal joint configs.

        This method:
          - Clears previous solutions.
          - Sets up start and goal states.
          - Runs the planner within allowed_time.
          - If solved, interpolates the path and converts it to a JointPath.

        Args:
            start (dict): Joint values for the start config.
            goal (dict): Joint values for the goal config.
            allowed_time (float, optional): Max planning time in seconds.
                Defaults to DEFAULT_PLANNING_TIME.

        Returns:
            tuple(bool, JointPath): A tuple with a success flag and the
            computed JointPath.
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
