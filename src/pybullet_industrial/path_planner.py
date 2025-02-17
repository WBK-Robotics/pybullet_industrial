import sys
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import CollisionChecker, RobotBase, JointPath

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
        self.setBounds(lower_limit, upper_limit)

    def setBounds(self, lower_limit, upper_limit):
        """
        Sets the bounds of the state space.

        Args:
            lower_limit (list): Lower limits for each dimension.
            upper_limit (list): Upper limits for each dimension.
        """
        bounds = ob.RealVectorBounds(self.getDimension())
        for i, joint in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint])
            bounds.setHigh(i, upper_limit[joint])
        super().setBounds(bounds)

    def getDimension(self):
        """
        Returns the dimension of the state space.

        Returns:
            int: The number of dimensions.
        """
        return super().getDimension()

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

    def create_multi_objective(self, weighted_objective_list: list):
        multi_optimization_objective = ob.MultiOptimizationObjective(self.si)

        for objective, weight in weighted_objective_list:
            multi_optimization_objective.addObjective(objective(self.si), weight)

        return multi_optimization_objective


class RobotPathClearanceObjective(ob.StateCostIntegralObjective):
    """
    Encourages paths with high clearance by using cost functions.

    Returns a reciprocal cost of the accumulated cost with a small offset
    to avoid division by zero.

    Args:
        si (RobotSpaceInformation): The space information instance.
        state_cost_functions (list, optional): List of cost functions.
    """

    def __init__(self, si: RobotSpaceInformation, collision_checker: CollisionChecker, clearance_distance: float = 0.0):
        super().__init__(si, True)
        self.si = si
        self.collision_checker = collision_checker
        self.clearance_distance = clearance_distance

    def stateCost(self, state: ob.State):
        """
        Computes the cost of the given state.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost.
        """
        self.si.set_state(state)
        min_distance = self.clearance_distance

        for (bodyA, bodyB), _ in self.collision_checker.external_collision_pairs:
            curr_distance = self.collision_checker.get_min_body_distance(bodyA, bodyB, self.clearance_distance)
            if curr_distance < min_distance:
                min_distance = curr_distance

        return ob.Cost(1 / (min_distance + sys.float_info.min))


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
                 constraint_functions=None,
                 objectives=None
                 ):
        self.robot = robot
        # Create robot-specific state space and space information.
        self.state_space = RobotStateSpace(robot)
        self.space_information = RobotSpaceInformation(self.state_space)

        # Attach the validity checker.
        validity_checker = RobotValidityChecker(
            self.space_information,
            collision_check_functions,
            constraint_functions
        )
        self.space_information.setStateValidityChecker(validity_checker)
        self.space_information.setup()
        super().__init__(self.space_information)

        optimization_objective = RobotOptimizationObjective(
            self.space_information
        )
        self.setOptimizationObjective(
            optimization_objective.create_multi_objective(objectives)
        )

        # Allocate the planner.
        optimizing_planner = self.allocate_planner(planner_name)
        self.setPlanner(optimizing_planner)

    def allocate_planner(self, plannerType: str):
        """
        Allocates an OMPL planner based on the given planner type.

        Args:
            plannerType (str): The desired planner type.

        Returns:
            An OMPL planner instance.
        """
        ptype = plannerType.lower()
        if ptype == "bfmtstar":
            return og.BFMT(self.space_information)
        elif ptype == "bitstar":
            return og.BITstar(self.space_information)
        elif ptype == "fmtstar":
            return og.FMT(self.space_information)
        elif ptype == "informedrrtstar":
            return og.InformedRRTstar(self.space_information)
        elif ptype == "prmstar":
            return og.PRMstar(self.space_information)
        elif ptype == "rrtstar":
            return og.RRTstar(self.space_information)
        elif ptype == "sorrtstar":
            return og.SORRTstar(self.space_information)
        else:
            ou.OMPL_ERROR("The specified planner type is not implemented.")
            return None

    def setStartAndGoalStates(self, start: ob.State, goal: ob.State):
        """
        Sets the start and goal states for the planning problem.

        Args:
            start (ob.State): The start state.
            goal (ob.State): The goal state.
        """
        start = self.state_space.dict_to_list(start)
        goal = self.state_space.dict_to_list(goal)
        start_state = self.space_information.list_to_state(start)
        goal_state = self.space_information.list_to_state(goal)
        super().setStartAndGoalStates(start_state, goal_state)

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

        self.clear()
        orig_state = start.copy()

        self.setStartAndGoalStates(start, goal)

        solved = self.solve(allowed_time)
        res = False
        joint_path = None

        if solved:
            sol_path = self.getSolutionPath()
            sol_path.interpolate(INTERPOLATE_NUM)
            states = sol_path.getStates()
            path_list = np.array([
                self.state_space.state_to_list(st)
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
