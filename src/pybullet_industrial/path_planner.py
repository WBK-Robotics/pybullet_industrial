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
    Constructs an OMPL RealVectorStateSpace based on the robot's joint limits.

    The number of dimensions is derived from the movable joints. If a joint
    limit is infinite, ±CONTINUOUS_JOINT_BOUNDS is used.

    Args:
        robot (RobotBase): The robot object providing joint info.
    """
    def __init__(self, robot: RobotBase):
        self.robot = robot
        # Get joint order (assumed to be the first element returned).
        self.joint_order = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order)
        )
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
        """
        Converts an OMPL state into a list of joint values.

        Args:
            state (ob.State): The OMPL state.

        Returns:
            list: Joint values in the order defined by joint_order.
        """
        return [state[i] for i in range(len(self.joint_order))]

    def dict_to_list(self, joint_dict: dict):
        """
        Converts a dictionary of joint values into an ordered list.

        Args:
            joint_dict (dict): Maps joint names to joint values.

        Returns:
            list: Joint values ordered as in joint_order.
        """
        return [joint_dict[joint] for joint in self.joint_order]


class RobotSpaceInformation(ob.SpaceInformation):
    """
    Wraps the RobotStateSpace to provide helper methods for state conversion
    and updating the robot configuration.

    Args:
        state_space (RobotStateSpace): The state space instance for the robot.
    """
    def __init__(self, state_space: RobotStateSpace):
        # Pass the state space to the parent constructor.
        super().__init__(state_space)
        self.robot = state_space.robot
        self.state_space = state_space

    def allocState(self):
        """
        Allocates a new state using the underlying state space.

        Returns:
            ob.State: A new state object.
        """
        return self.state_space.allocState()

    def list_to_state(self, joint_values: list):
        """
        Converts a list of joint values into an OMPL state.

        Args:
            joint_values (list): Joint values following the joint order.

        Returns:
            ob.State: The corresponding state object.
        """
        state = ob.State(self.state_space)
        for i, value in enumerate(joint_values):
            state[i] = value
        return state

    def set_state(self, state: ob.State):
        """
        Updates the robot's configuration based on the given state.

        Collision/constraint checking is assumed to be handled elsewhere.

        Args:
            state (ob.State): The state used to update the robot.
        """
        joint_positions = self.state_space.state_to_list(state)
        self.robot.reset_joint_position(
            dict(zip(self.state_space.joint_order, joint_positions)),
            True
        )

    def setStateValidityChecker(self, validity_checker):
        """
        Sets the state validity checker.

        Args:
            validity_checker: An object to check state validity.
        """
        super().setStateValidityChecker(validity_checker)

    def setup(self):
        """
        Performs any necessary setup steps for the space information.
        """
        super().setup()


class RobotValidityChecker(ob.StateValidityChecker):
    """
    Checks if a state is valid using collision and constraint functions.

    Args:
        space_information (RobotSpaceInformation): The space info instance.
        collision_check_functions (list): List of collision-check functions.
        constraint_functions (list, optional): List of constraint functions.
    """
    def __init__(self, space_information: RobotSpaceInformation,
                 collision_check_functions: list,
                 constraint_functions=None):
        # Initialize the parent with the space information.
        super().__init__(space_information)
        self.space_information = space_information
        self.collision_check_functions = collision_check_functions
        self.constraint_functions = constraint_functions

    def isValid(self, state: ob.State):
        """
        Determines if the given state is valid.

        Updates the robot configuration before checking collisions and
        constraints.

        Args:
            state (ob.State): The state to be validated.

        Returns:
            bool: True if valid; False otherwise.
        """
        self.space_information.set_state(state)
        if self.constraint_functions:
            for constraint in self.constraint_functions:
                if not constraint():
                    return False
        for collision_check in self.collision_check_functions:
            if not collision_check():
                return False
        return True


class RobotProblemDefinition(ob.ProblemDefinition):
    """
    Defines the planning problem using the robot's space information.

    Provides helper methods to set start/goal states, clear solution paths,
    and set optimization objectives.

    Args:
        space_information (RobotSpaceInformation): The space info instance.
    """
    def __init__(self, space_information: RobotSpaceInformation):
        super().__init__(space_information)
        self.space_information = space_information

    def clearSolutionPaths(self):
        """
        Clears any previously stored solution paths.
        """
        super().clearSolutionPaths()

    def getSolutionPath(self):
        """
        Retrieves the current solution path.

        Returns:
            The solution path as provided by the OMPL library.
        """
        return super().getSolutionPath()

    def setOptimizationObjective(self, optimization_objective):
        """
        Sets the optimization objective for the planning problem.

        Args:
            optimization_objective: The objective to optimize.
        """
        super().setOptimizationObjective(optimization_objective)

    def setStartAndGoalStates(self, start: dict, goal: dict):
        """
        Sets start and goal states using the robot's space information.

        Converts the input dictionaries to states and registers them.

        Args:
            start (dict): Start joint configuration.
            goal (dict): Goal joint configuration.
        """
        rss = self.space_information.state_space
        start_state_list = rss.dict_to_list(start)
        goal_state_list = rss.dict_to_list(goal)
        start_state = self.space_information.list_to_state(start_state_list)
        goal_state = self.space_information.list_to_state(goal_state_list)
        super().setStartAndGoalStates(start_state, goal_state)


class RobotOptimizationObjective(ob.OptimizationObjective):
    """
    Factory class for creating robot-specific optimization objectives.

    Provides an interface to create various objective types.

    Args:
        si (RobotSpaceInformation): The space information instance.
    """
    def __init__(self, si: RobotSpaceInformation):
        super().__init__(si)
        self.si = si

    @classmethod
    def create(cls, si: RobotSpaceInformation, objectiveType: str,
               state_cost_functions=None):
        """
        Creates an optimization objective based on the given type.

        Available objective types (case-insensitive):
          - "pathclearance": Uses a robot-specific clearance objective.
          - "pathlength": Uses OMPL's PathLengthOptimizationObjective.
          - "thresholdpathlength": PathLength with a cost threshold.
          - "weightedlengthandclearancecombo": Multi-objective combining
            length and clearance.

        Args:
            si (RobotSpaceInformation): The space info instance.
            objectiveType (str): The objective type.
            state_cost_functions (list, optional): Cost functions for state.

        Returns:
            An optimization objective instance.
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
            ou.OMPL_ERROR("The specified optimization objective is not "
                          "implemented.")
            return None


class RobotPathClearanceObjective(ob.StateCostIntegralObjective):
    """
    Encourages paths with high clearance by using cost functions.

    Returns a reciprocal cost of the accumulated cost with a small offset
    to avoid division by zero.

    Args:
        si (RobotSpaceInformation): The space information instance.
        state_cost_functions (list, optional): List of cost functions.
    """
    def __init__(self, si: RobotSpaceInformation, state_cost_functions=None):
        super().__init__(si, True)
        self.si = si
        self.state_cost_functions = state_cost_functions

    def stateCost(self, state: ob.State):
        """
        Computes the cost of the given state.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost.
        """
        self.si.set_state(state)
        total_cost = 0
        if self.state_cost_functions:
            for state_cost in self.state_cost_functions:
                total_cost += state_cost()
        return ob.Cost(1 / (total_cost + sys.float_info.min))


class RobotPlanner(ob.Planner):
    """
    A wrapper for OMPL planners that selects and wraps a planner type.

    Args:
        si (RobotSpaceInformation): The space information instance.
        plannerType (str): The type of planner to allocate.
    """
    def __init__(self, si: RobotSpaceInformation, plannerType: str):
        # Initialize the base planner with space information and name.
        super().__init__(si, plannerType)
        self.si = si
        self.planner = self._allocate_planner(plannerType)

    def _allocate_planner(self, plannerType: str):
        """
        Allocates an OMPL planner based on the given planner type.

        Args:
            plannerType (str): The desired planner type.

        Returns:
            An OMPL planner instance.
        """
        ptype = plannerType.lower()
        if ptype == "bfmtstar":
            return og.BFMT(self.si)
        elif ptype == "bitstar":
            return og.BITstar(self.si)
        elif ptype == "fmtstar":
            return og.FMT(self.si)
        elif ptype == "informedrrtstar":
            return og.InformedRRTstar(self.si)
        elif ptype == "prmstar":
            return og.PRMstar(self.si)
        elif ptype == "rrtstar":
            return og.RRTstar(self.si)
        elif ptype == "sorrtstar":
            return og.SORRTstar(self.si)
        else:
            ou.OMPL_ERROR("The specified planner type is not implemented.")
            return None

    def setProblemDefinition(self, pdef: ob.ProblemDefinition):
        """
        Sets the problem definition for the planner.

        Args:
            pdef (ob.ProblemDefinition): The problem definition.
        """
        self.planner.setProblemDefinition(pdef)

    def clear(self):
        """
        Clears the planner's current configuration.
        """
        self.planner.clear()

    def setup(self):
        """
        Performs any necessary setup for the planner.
        """
        self.planner.setup()

    def solve(self, allowed_time: float):
        """
        Attempts to solve the planning problem within allowed_time.

        Args:
            allowed_time (float): The maximum planning time in seconds.

        Returns:
            bool: True if a solution was found, False otherwise.
        """
        return self.planner.solve(allowed_time)


class PathPlanner(og.SimpleSetup):
    """
    Sets up the entire planning problem using robot-specific classes.

    Creates the state space, space information, validity checker, and problem
    definition, then allocates and runs an OMPL planner.

    Args:
        robot (RobotBase): The robot to plan for.
        collision_check_functions (list): Collision-check functions.
        planner_name (str): The name of the planner to use.
        objective (str): The optimization objective.
        constraint_functions (list, optional): Constraint-check functions.
        state_cost_functions (list, optional): State cost functions.
    """
    def __init__(self, robot: RobotBase,
                 collision_check_functions: list,
                 planner_name: str = "BITstar",
                 objective: str = "PathLength",
                 constraint_functions=None,
                 state_cost_functions=None):
        self.robot = robot
        # Create robot-specific state space and space information.
        self.state_space = RobotStateSpace(robot)
        self.space_information = RobotSpaceInformation(self.state_space)

        # Attach the validity checker.
        self.validity_checker = RobotValidityChecker(
            self.space_information,
            collision_check_functions,
            constraint_functions
        )
        self.space_information.setStateValidityChecker(self.validity_checker)
        self.space_information.setup()
        super().__init__(self.space_information)


        # # Define the planning problem.
        # self.problem_definition = RobotProblemDefinition(
        #     self.space_information
        # )
        # # Create the optimization objective.
        # optimization_objective = RobotOptimizationObjective.create(
        #     self.space_information, objective, state_cost_functions
        # )
        # self.problem_definition.setOptimizationObjective(
        #     optimization_objective
        # )

        # Allocate the planner.
        self.planner = RobotPlanner(self.space_information, planner_name)
        self.setPlanner(self.planner)

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = DEFAULT_PLANNING_TIME):
        """
        Plans a path between given start and goal joint configurations.

        If a solution is found, the path is interpolated and returned as a
        JointPath.

        Args:
            start (dict): The start joint configuration.
            goal (dict): The goal joint configuration.
            allowed_time (float): Maximum planning time in seconds.

        Returns:
            tuple: (bool, JointPath) where bool indicates success.
        """

        orig_state = start.copy()
        start = self.state_space.dict_to_list(start)
        goal = self.state_space.dict_to_list(goal)
        start_state = self.space_information.list_to_state(start)
        goal_state = self.space_information.list_to_state(goal)
        self.setStartAndGoalStates(start_state, goal_state)

        solved = self.solve(allowed_time)
        res = False
        joint_path = None

        if solved:
            sol_path = self.getSolutionPath()
            sol_path.interpolate(INTERPOLATE_NUM)
            states = sol_path.getStates()
            path_list = np.array([
                self.space_information.state_space.state_to_list(st)
                for st in states
            ])
            joint_path = JointPath(
                path_list.transpose(),
                tuple(self.state_space.joint_order)
            )
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_state, True)
        return res, joint_path
