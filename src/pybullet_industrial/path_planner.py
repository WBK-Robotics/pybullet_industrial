import sys
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import (CollisionChecker, RobotBase, JointPath)
import numpy as np
import pybullet as p

# Global constant for continuous joint bounds.
CONTINUOUS_JOINT_BOUNDS = 3 * np.pi

INTERPOLATE_NUM = 500  # Number of segments for interpolating the path
DEFAULT_PLANNING_TIME = 5.0  # Maximum planning time (seconds)


class SamplingSpace:
    """Creates an OMPL state space based on robot joint limits and provides
    utilities for state conversion and creation.

    This class always creates a RealVectorStateSpace. For each joint,
    if the joint limit is infinite the bounds are set to
    [-CONTINUOUS_JOINT_BOUNDS, CONTINUOUS_JOINT_BOUNDS]. Otherwise, a small
    margin is added to the provided limits.

    It also provides a helper to create start and goal states.

    Args:
        robot (RobotBase): The robot model with its kinematics.
    """

    def __init__(self, robot: RobotBase):
        # Retrieve joint limits as two dictionaries (lower and upper bounds).
        self.joint_order = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order)
        )
        self.robot = robot

        # Always use a RealVectorStateSpace.
        self.real_vector = True
        self.space = self.build_realvector_space(lower_limit, upper_limit)

    def build_realvector_space(self, lower_limit: dict, upper_limit: dict):
        """Builds a RealVectorStateSpace with special handling for
        continuous joints.

        For each joint:
          - If a limit is infinite, set bounds to
            [-CONTINUOUS_JOINT_BOUNDS, CONTINUOUS_JOINT_BOUNDS].
          - Otherwise, use the provided limits with a small margin (±0.1).

        Args:
            lower_limit (dict): Lower limits for each joint.
            upper_limit (dict): Upper limits for each joint.

        Returns:
            ob.RealVectorStateSpace: The constructed state space.
        """
        num_dims = len(self.joint_order)
        bounds = ob.RealVectorBounds(num_dims)
        for i, joint in enumerate(self.joint_order):
            if np.isinf(lower_limit[joint]) or np.isinf(upper_limit[joint]):
                bounds.setLow(i, -CONTINUOUS_JOINT_BOUNDS)
                bounds.setHigh(i, CONTINUOUS_JOINT_BOUNDS)
            else:
                bounds.setLow(i, lower_limit[joint] - 0.1)
                bounds.setHigh(i, upper_limit[joint] + 0.1)
        space = ob.RealVectorStateSpace(num_dims)
        space.setBounds(bounds)
        return space

    def realvector_state_to_list(self, state: ob.State):
        """Converts a RealVector OMPL state into a list of joint values.

        Args:
            state (ob.State): An OMPL state object.

        Returns:
            list: The joint values extracted from the state.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]

    def get_space(self):
        """Returns the constructed OMPL state space.

        Returns:
            ob.StateSpace: The state space used for planning.
        """
        return self.space

    def set_state(self, state: ob.State):
        """Updates the robot's joint positions to match the given state.

        This method resets the robot joints to the values specified in
        the state. It does not perform collision or constraint checks.

        Args:
            state (ob.State): The state containing joint values.
        """
        # Extract joint positions from the state.
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        self.robot.reset_joint_position(
            dict(zip(self.joint_order, joint_positions)), True
        )

    def set_start_goal_states(self, start: dict, goal: dict):
        """Creates and returns the start and goal states for planning.

        Args:
            start (dict): Joint values for the start configuration.
            goal (dict): Joint values for the goal configuration.

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
    """An objective that encourages paths with high clearance from obstacles.

    The cost is computed as the reciprocal of the state's clearance plus a
    small constant, which favors paths that stay away from obstacles.

    Args:
        si (ob.SpaceInformation): The space information.
        sampling_space (SamplingSpace): The sampling space instance.
        state_cost_function (iterable of callables, optional): Functions to
            compute additional state costs.
    """

    def __init__(self, si: ob.SpaceInformation,
                 sampling_space: SamplingSpace,
                 state_cost_function=None):
        super(ClearanceObjective, self).__init__(si, True)
        self.sampling_space = sampling_space
        self.state_cost_function = state_cost_function

    def stateCost(self, state: ob.State):
        """Computes the cost of a state based on its clearance.

        Args:
            state (ob.State): The state for cost evaluation.

        Returns:
            ob.Cost: The computed cost.
        """
        # Update the robot's joint positions to match the state.
        self.sampling_space.set_state(state)
        total_cost = 0
        # Sum cost contributions from all state cost functions.
        for state_cost in self.state_cost_function:
            total_cost += state_cost()

        return ob.Cost(1 / (total_cost + sys.float_info.min))


class ValidityChecker(ob.StateValidityChecker):
    """Checks if a state is valid with respect to collisions and constraints.

    This checker updates the robot's configuration to the given state,
    then verifies that no collisions occur and that all additional
    constraint functions are satisfied.

    Args:
        space_information (ob.SpaceInformation): The state space info.
        sampling_space (SamplingSpace): The sampling space instance.
        collision_checker_list (list): List of collision checker objects.
        constraint_functions (iterable of callables, optional): Functions to
            evaluate extra state constraints.
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
        """Determines whether the given state is valid.

        A state is valid if it is collision-free and satisfies all
        constraint functions.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if valid, False otherwise.
        """
        # Update the robot's joint positions to the provided state.
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
    """Plans motion for a robot in a constrained environment.

    This class configures the state space, sets up the planning problem,
    and computes a path between start and goal states.

    Args:
        robot (RobotBase): The robot instance for planning.
        collision_checker_list (list): List of collision checker objects.
        planner_name (str, optional): The planner to use (e.g., "BITstar").
            Defaults to "BITstar".
        objective (str, optional): The optimization objective (e.g.,
            "PathLength"). Defaults to "PathLength".
        constraint_functions (iterable of callables, optional): Functions to
            evaluate extra state constraints.
        state_cost (iterable of callables, optional): Functions to compute state
            costs.
    """

    def __init__(self, robot: RobotBase, collision_checker_list: list,
                 planner_name: str = "BITstar",
                 objective: str = "PathLength", constraint_functions=None,
                 state_cost=None):
        self.state_cost_function = state_cost
        self.robot = robot
        self.collision_checker_list = collision_checker_list

        # Create the state space using SamplingSpace.
        self.sampling_space = SamplingSpace(robot)
        self.space = self.sampling_space.get_space()
        self.real_vector = self.sampling_space.real_vector

        # Set up OMPL space information and validity checking.
        self.space_information = ob.SpaceInformation(self.space)
        self.validity_checker = ValidityChecker(
            self.space_information,
            self.sampling_space,
            collision_checker_list,
            constraint_functions
        )
        self.space_information.setStateValidityChecker(self.validity_checker)
        self.space_information.setup()

        # Configure the problem definition and objective.
        self.problem_definition = ob.ProblemDefinition(self.space_information)
        self.problem_definition.setOptimizationObjective(
            self.allocateObjective(objective)
        )

        # Allocate the desired planner.
        self.planner = self.allocatePlanner(planner_name)

    def allocateObjective(self, objectiveType: str):
        """Selects and allocates an optimization objective.

        Args:
            objectiveType (str): The type of optimization objective.

        Returns:
            ob.OptimizationObjective: The allocated objective.
        """
        if objectiveType.lower() == "pathclearance":
            return ClearanceObjective(self.space_information,
                                      self.sampling_space,
                                      self.state_cost_function)
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
        elif objectiveType.lower() == "weightedlengthandclearancecombo":
            length_obj = ob.PathLengthOptimizationObjective(
                self.space_information
            )
            clear_obj = ClearanceObjective(self.space_information,
                                           self.sampling_space)
            opt = ob.MultiOptimizationObjective(self.space_information)
            opt.addObjective(length_obj, 5.0)
            opt.addObjective(clear_obj, 1.0)
            return opt
        else:
            ou.OMPL_ERROR(
                "Optimization-objective is not implemented in allocation "
                "function."
            )

    def allocatePlanner(self, plannerType: str):
        """Allocates and returns a planner based on the specified type.

        Args:
            plannerType (str): The name of the planner to use.

        Returns:
            og.Planner: The allocated planner.
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
                "Planner-type is not implemented in allocation function."
            )

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = DEFAULT_PLANNING_TIME):
        """Plans a path between the given start and goal states.

        This method creates start and goal states, runs the planner, and
        returns a success flag and the corresponding joint path.

        Args:
            start (dict): Joint values for the start configuration.
            goal (dict): Joint values for the goal configuration.
            allowed_time (float, optional): Maximum planning time (seconds).
                Defaults to DEFAULT_PLANNING_TIME.

        Returns:
            tuple(bool, JointPath): A tuple containing a success flag and
            the joint path.
        """
        orig_state = start
        # Create start and goal states using the SamplingSpace helper.
        s, g = self.sampling_space.set_start_goal_states(start, goal)
        self.problem_definition.setStartAndGoalStates(s, g)

        self.planner.setProblemDefinition(self.problem_definition)
        self.planner.setup()

        solved = self.planner.solve(allowed_time)
        res = False
        joint_path = None
        if solved:
            sol_path = self.problem_definition.getSolutionPath()
            sol_path.interpolate(INTERPOLATE_NUM)
            states = sol_path.getStates()
            path_list = np.array(
                [self.sampling_space.realvector_state_to_list(st)
                 for st in states]
            )
            joint_path = JointPath(
                path_list.transpose(),
                tuple(self.robot.get_moveable_joints()[0])
            )
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_state, True)
        return res, joint_path
