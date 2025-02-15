import sys
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import CollisionChecker, RobotBase, JointPath

# Global constant for continuous joint bounds.
CONTINUOUS_JOINT_BOUNDS = 3 * np.pi
# Number of segments to interpolate along the planned path.
INTERPOLATE_NUM = 500
# Maximum allowed planning time in seconds.
DEFAULT_PLANNING_TIME = 5.0


class RobotStateSpace(ob.RealVectorStateSpace):
    """
    Constructs an OMPL RealVectorStateSpace from the robot's joint limits.
    It computes the number of dimensions from the robot’s movable joints and
    sets each bound. If a joint limit is infinite, it uses ±CONTINUOUS_JOINT_BOUNDS.
    """
    def __init__(self, robot: RobotBase):
        self.robot = robot
        # Get the joint order (assumed to be the first element returned).
        self.joint_order = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits(set(self.joint_order))
        num_dims = len(self.joint_order)
        # Initialize the underlying RealVectorStateSpace.
        super().__init__(num_dims)
        bounds = ob.RealVectorBounds(num_dims)
        for i, joint in enumerate(self.joint_order):
            if np.isinf(lower_limit[joint]) or np.isinf(upper_limit[joint]):
                bounds.setLow(i, -CONTINUOUS_JOINT_BOUNDS)
                bounds.setHigh(i, CONTINUOUS_JOINT_BOUNDS)
            else:
                bounds.setLow(i, lower_limit[joint])
                bounds.setHigh(i, upper_limit[joint])
        self.setBounds(bounds)

    def state_to_list(self, state: ob.State):
        """Converts an OMPL state to a list of joint values."""
        return [state[i] for i in range(len(self.joint_order))]


class RobotSpaceInformation(ob.SpaceInformation):
    """
    Wraps the RobotStateSpace and the robot to provide helper methods
    for state conversion and updating the robot configuration.
    """
    def __init__(self, state_space: RobotStateSpace, robot: RobotBase):
        # Pass the state space to the parent (ob.SpaceInformation).
        super().__init__(state_space)
        self.robot = robot
        self.state_space = state_space

    def allocState(self):
        """Delegate state allocation to the underlying state space."""
        return self.state_space.allocState()

    def state_to_list(self, state: ob.State):
        """Convert an OMPL state to a list of joint values."""
        return self.state_space.state_to_list(state)

    def set_state(self, state: ob.State):
        """
        Updates the robot's configuration based on the given state.
        (Collision/constraint checking is assumed to be done elsewhere.)
        """
        joint_positions = self.state_to_list(state)
        self.robot.reset_joint_position(
            dict(zip(self.state_space.joint_order, joint_positions)), True
        )

    def set_start_goal_states(self, start: dict, goal: dict):
        """
        Creates start and goal OMPL states from dictionaries mapping joint names to values.
        """
        start_state = self.allocState()
        goal_state = self.allocState()
        for i, joint in enumerate(self.state_space.joint_order):
            start_state[i] = start[joint]
            goal_state[i] = goal[joint]
        return start_state, goal_state


class RobotValidityChecker(ob.StateValidityChecker):
    """
    Uses the robot's space information along with collision and constraint functions
    to determine if a state is valid.
    """
    def __init__(self, space_information: RobotSpaceInformation,
                 collision_check_functions: list,
                 constraint_functions=None):
        # Pass the RobotSpaceInformation (a subclass of ob.SpaceInformation) to the parent.
        super().__init__(space_information)
        self.space_information = space_information
        self.collision_check_functions = collision_check_functions
        self.constraint_functions = constraint_functions

    def isValid(self, state: ob.State):
        # Update the robot to match the state.
        self.space_information.set_state(state)
        # Check any extra constraints.
        if self.constraint_functions:
            for constraint in self.constraint_functions:
                if not constraint():
                    return False
        # Check collisions.
        for collision_check in self.collision_check_functions:
            if not collision_check():
                return False
        return True


class RobotProblemDefinition(ob.ProblemDefinition):
    """
    Wraps OMPL's ProblemDefinition and provides a helper for setting
    start and goal states using the robot's space information.
    """
    def __init__(self, space_information: RobotSpaceInformation):
        super().__init__(space_information)
        self.space_information = space_information

    def set_start_goal_states(self, start: dict, goal: dict):
        start_state, goal_state = self.space_information.set_start_goal_states(start, goal)
        self.setStartAndGoalStates(start_state, goal_state)


class RobotOptimizationObjective(ob.OptimizationObjective):
    """
    A factory class for creating robot-specific optimization objectives.
    This class inherits from ob.OptimizationObjective and provides a common
    interface for creating various objective types.
    """
    def __init__(self, si: RobotSpaceInformation):
        super().__init__(si)
        self.si = si

    @classmethod
    def create(cls, si: RobotSpaceInformation, objectiveType: str, state_cost_functions=None):
        """
        Factory method to create an optimization objective based on the specified type.

        Available objective types (case-insensitive):
          - "pathclearance": uses a robot-specific clearance objective.
          - "pathlength": uses OMPL's PathLengthOptimizationObjective.
          - "thresholdpathlength": a PathLength objective with a cost threshold.
          - "weightedlengthandclearancecombo": a multi-objective combining length and clearance.
        """
        if objectiveType.lower() == "pathclearance":
            return RobotPathClearanceObjective(si, state_cost_functions)
        elif objectiveType.lower() == "pathlength":
            return ob.PathLengthOptimizationObjective(si)
        elif objectiveType.lower() == "thresholdpathlength":
            obj = ob.PathLengthOptimizationObjective(si)
            obj.setCostThreshold(ob.Cost(1.51))
            return obj
        elif objectiveType.lower() == "weightedlengthandclearancecombo":
            length_obj = ob.PathLengthOptimizationObjective(si)
            clear_obj = RobotPathClearanceObjective(si, state_cost_functions)
            opt = ob.MultiOptimizationObjective(si)
            opt.addObjective(length_obj, 5.0)
            opt.addObjective(clear_obj, 1.0)
            return opt
        else:
            ou.OMPL_ERROR("The specified optimization objective is not implemented.")
            return None


class RobotPathClearanceObjective(ob.StateCostIntegralObjective):
    """
    An optimization objective that encourages paths with high clearance.
    It uses user-provided cost functions and returns a cost that is the reciprocal
    of the accumulated cost (with a small constant added to avoid division by zero).
    """
    def __init__(self, si: RobotSpaceInformation, state_cost_functions=None):
        super().__init__(si, True)
        self.si = si
        self.state_cost_functions = state_cost_functions

    def stateCost(self, state: ob.State):
        self.si.set_state(state)
        total_cost = 0
        if self.state_cost_functions:
            for state_cost in self.state_cost_functions:
                total_cost += state_cost()
        # Return reciprocal cost (adding a small constant to avoid division by zero)
        return ob.Cost(1 / (total_cost + sys.float_info.min))


class PathPlanner:
    """
    Sets up the planning problem using the robot-specific classes.
    It creates the state space, space information, validity checker, and problem definition,
    and then allocates and runs an OMPL planner.
    """
    def __init__(self, robot: RobotBase,
                 collision_check_functions: list,
                 planner_name: str = "BITstar",
                 objective: str = "PathLength",
                 constraint_functions=None,
                 state_cost_functions=None):

        self.robot = robot

        # Create the robot-specific state space and space information.
        self.state_space = RobotStateSpace(robot)
        self.space_information = RobotSpaceInformation(self.state_space, robot)

        # Attach the validity checker.
        self.validity_checker = RobotValidityChecker(
            self.space_information,
            collision_check_functions,
            constraint_functions
        )
        self.space_information.setStateValidityChecker(self.validity_checker)
        self.space_information.setup()

        # Define the planning problem with the custom problem definition.
        self.problem_definition = RobotProblemDefinition(self.space_information)
        # Create the optimization objective using the new factory.
        optimization_objective = RobotOptimizationObjective.create(
            self.space_information, objective, state_cost_functions
        )
        self.problem_definition.setOptimizationObjective(optimization_objective)

        # Allocate the planner.
        self.planner = self.allocate_planner(planner_name)

    def allocate_planner(self, plannerType: str):
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
            ou.OMPL_ERROR("The specified planner type is not implemented.")

    def plan_start_goal(self, start: dict, goal: dict, allowed_time: float = DEFAULT_PLANNING_TIME):
        """
        Plans a path between the given start and goal joint configurations.
        If a solution is found, it interpolates the path and returns a JointPath.
        """
        self.problem_definition.clearSolutionPaths()
        orig_state = start.copy()
        self.problem_definition.set_start_goal_states(start, goal)

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
            path_list = np.array([self.space_information.state_to_list(st) for st in states])
            joint_path = JointPath(
                path_list.transpose(),
                tuple(self.state_space.joint_order)
            )
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_state, True)
        return res, joint_path
