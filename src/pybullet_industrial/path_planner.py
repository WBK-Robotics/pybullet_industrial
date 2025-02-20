import numpy as np
from ompl import base as ob
from ompl import geometric as og
from pybullet_industrial import (CollisionChecker, RobotBase,
                                 JointPath)


class RobotStateSpace(ob.RealVectorStateSpace):
    """
    An OMPL state space that represents the robot's joint configuration,
    using the robot's joint limits for bounds.

    Attributes:
        robot (RobotBase): The robot instance.
        joint_order (list): The ordered list of movable joints.
    """

    def __init__(self, robot: RobotBase) -> None:
        """
        Initializes the state space based on the robot's joint limits.

        Args:
            robot (RobotBase): The robot object providing joint info.
        """
        self.robot: RobotBase = robot
        # Get the joint order (assumed to be the first element returned).
        self.joint_order: list = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order))
        num_dims: int = len(self.joint_order)
        # Initialize the base RealVectorStateSpace with the dimension.
        super().__init__(num_dims)
        self.set_bounds(lower_limit, upper_limit)

    def state_to_list(self, state: ob.State) -> list:
        """
        Converts an OMPL state to a list of joint values following the
        joint order.

        Args:
            state (ob.State): The OMPL state.

        Returns:
            list: List of joint values.
        """
        return [state[i] for i in range(len(self.joint_order))]

    def dict_to_list(self, joint_dict: dict) -> list:
        """
        Converts a dictionary mapping joint names to values into an
        ordered list according to joint_order.

        Args:
            joint_dict (dict): Dictionary of joint values.

        Returns:
            list: Ordered joint values.
        """
        return [joint_dict[joint] for joint in self.joint_order]

    def set_bounds(self, lower_limit: list, upper_limit: list) -> None:
        """
        Configures the bounds of the state space based on joint limits.

        Args:
            lower_limit (list): Lower limits for each joint.
            upper_limit (list): Upper limits for each joint.
        """
        bounds = ob.RealVectorBounds(self.getDimension())
        for i, joint in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint])
            bounds.setHigh(i, upper_limit[joint])
        super().setBounds(bounds)

    def get_dimension(self) -> int:
        """
        Retrieves the dimension (number of joints) of the state space.
        """
        return super().getDimension()


class RobotSpaceInformation(ob.SpaceInformation):
    """
    Provides robot-specific extensions to OMPL's SpaceInformation,
    including state conversion and robot configuration update.

    Attributes:
        robot (RobotBase): The robot instance.
        state_space (RobotStateSpace): The custom state space.
        endeffector: Optional endeffector for matching poses.
        moved_object: Optional object moved by the robot.
    """

    def __init__(self, state_space: RobotStateSpace,
                 endeffector=None,
                 moved_object=None,
                 validity_resolution: float = 0.0001) -> None:
        """
        Initializes the RobotSpaceInformation instance.

        Args:
            state_space (RobotStateSpace): The robot's state space.
            endeffector: Optional endeffector instance.
            moved_object: Optional moved object instance.
            validity_resolution (float): Resolution for state validity.
        """
        super().__init__(state_space)
        self.setStateValidityCheckingResolution(validity_resolution)
        self.robot: RobotBase = state_space.robot
        self.state_space: RobotStateSpace = state_space
        self.endeffector = endeffector
        self.moved_object = moved_object

    def list_to_state(self, joint_values: list) -> ob.State:
        """
        Converts a list of joint values into an OMPL state.

        Args:
            joint_values (list): Joint values in order.

        Returns:
            ob.State: The corresponding OMPL state.
        """
        state = ob.State(self.state_space)
        for i, value in enumerate(joint_values):
            state[i] = value
        return state

    def set_state(self, state: ob.State) -> None:
        """
        Updates the robot's configuration based on the provided state.
        Assumes that collision and constraint checking occur elsewhere.

        Args:
            state (ob.State): The state to apply.
        """
        joint_positions = self.state_space.state_to_list(state)
        self.robot.reset_joint_position(
            dict(zip(self.state_space.joint_order, joint_positions)),
            True
        )
        # Update endeffector and moved object if provided.
        if self.endeffector:
            self.endeffector.match_endeffector_pose(self.robot)
            if self.moved_object:
                self.endeffector.match_moving_object(self.moved_object)

    def set_state_validity_checking_resolution(
            self, resolution: float) -> None:
        """
        Sets the resolution for state validity checking.

        Args:
            resolution (float): The resolution value.
        """
        super().setStateValidityCheckingResolution(resolution)

    def set_state_validity_checker(self, validity_checker) -> None:
        """
        Registers a state validity checker.

        Args:
            validity_checker: The checker instance.
        """
        super().setStateValidityChecker(validity_checker)

    def setup(self) -> None:
        """
        Finalizes the setup of the space information.
        """
        super().setup()


class RobotValidityChecker(ob.StateValidityChecker):
    """
    Checks the validity of a state using collision and constraint tests.

    Attributes:
        space_information (RobotSpaceInformation): The robot's space info.
        collision_check_functions (list): Functions to check collisions.
        constraint_functions (list): Functions to check constraints.
    """

    def __init__(self, space_information: RobotSpaceInformation,
                 collision_check_functions: list,
                 constraint_functions: list = None) -> None:
        """
        Initializes the validity checker.

        Args:
            space_information (RobotSpaceInformation): The space info.
            collision_check_functions (list): Collision-check functions.
            constraint_functions (list, optional): Constraint functions.
        """
        super().__init__(space_information)
        self.space_information: RobotSpaceInformation = space_information
        self.collision_check_functions: list = collision_check_functions
        self.constraint_functions: list = constraint_functions

    def isValid(self, state: ob.State) -> bool:
        """
        Determines if a state is valid by updating the robot's state
        and running collision and constraint tests.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if valid, False otherwise.
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


class RobotMultiOptimizationObjective(ob.MultiOptimizationObjective):
    """
    Aggregates multiple robot-specific optimization objectives.

    Attributes:
        si (RobotSpaceInformation): The robot's space information.
    """

    def __init__(self, si: RobotSpaceInformation,
                 weighted_objective_list: list) -> None:
        """
        Initializes the multi-objective.

        Args:
            si (RobotSpaceInformation): The space info.
            weighted_objective_list (list): List of (objective, weight) pairs.
        """
        super().__init__(si)
        self.si: RobotSpaceInformation = si
        self.update_objective(weighted_objective_list)

    def update_objective(self, weighted_objective_list: list) -> None:
        """
        Updates the multi-objective with provided weighted objectives.

        Args:
            weighted_objective_list (list): (objective, weight) pairs.
        """
        for objective, weight in weighted_objective_list:
            self.add_objective(objective(self.si), weight)

    def add_objective(self, objective, weight: float) -> None:
        """
        Adds a single objective with its weight to the multi-objective.

        Args:
            objective: The objective to add.
            weight (float): The weight assigned.
        """
        super().addObjective(objective, weight)


class RobotPathClearanceObjective(ob.StateCostIntegralObjective):
    """
    Defines a cost objective that rewards paths with higher clearance.
    The cost penalizes low clearances between the robot and obstacles.
    """

    def __init__(self, si: RobotSpaceInformation,
                 collision_checker: CollisionChecker,
                 clearance_distance: float = 0.0) -> None:
        """
        Initializes the clearance objective.

        Args:
            si (RobotSpaceInformation): The robot's space info.
            collision_checker (CollisionChecker): The collision checker.
            clearance_distance (float): Base clearance distance.
        """
        super().__init__(si, True)
        self.si: RobotSpaceInformation = si
        self.collision_checker: CollisionChecker = collision_checker
        self.clearance_distance: float = clearance_distance

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        Computes the cost for a state based on its proximity to obstacles.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost.
        """
        self.si.set_state(state)
        min_distance: float = self.clearance_distance
        # Determine the minimum clearance over all collision pairs.
        for (bodyA, bodyB), _ in (
                self.collision_checker.external_collision_pairs):
            curr_distance = self.collision_checker.get_min_body_distance(
                bodyA, bodyB, self.clearance_distance)
            if curr_distance < 0:
                curr_distance = 0
            if curr_distance < min_distance:
                min_distance = curr_distance
        return ob.Cost(self.clearance_distance - min_distance)


class RobotPlannerSimpleSetup(og.SimpleSetup):
    """
    Integrates all components to define and solve a robot planning problem.
    Sets up the state space, space information, validity checker,
    optimization objective, and planner. Provides an API to plan a path
    between start and goal joint configurations.
    """

    def __init__(self, robot: RobotBase,
                 collision_check_functions: list,
                 planner_type,
                 interpolation_precision: float = 0.001,
                 constraint_functions: list = None,
                 objectives: list = None,
                 endeffector=None,
                 moved_object=None) -> None:
        """
        Initializes the planning problem.

        Args:
            robot (RobotBase): The robot instance.
            collision_check_functions (list): Collision-check functions.
            planner_type: The planner class/type to use.
            interpolation_precision (float): Spacing for interpolation.
            constraint_functions (list, optional): Constraint functions.
            objectives (list, optional): List of optimization objectives.
            endeffector: Optional endeffector instance.
            moved_object: Optional moved object instance.
        """
        self.setup_space_information(robot, collision_check_functions,
                                     constraint_functions, endeffector,
                                     moved_object)
        if objectives:
            self.setOptimizationObjective(objectives)
        self.set_interpolation_precision(interpolation_precision)
        self.planner_type = planner_type
        self.setPlanner(planner_type)

    def setup_space_information(self, robot: RobotBase,
                                collision_check_functions: list,
                                constraint_functions: list = None,
                                endeffector=None,
                                moved_object=None) -> None:
        """
        Initializes the state space and configures the validity checker.

        Args:
            robot (RobotBase): The robot instance.
            collision_check_functions (list): Collision-check functions.
            constraint_functions (list, optional): Constraint functions.
            endeffector: Optional endeffector instance.
            moved_object: Optional moved object instance.
        """
        self.robot = robot
        self.state_space = RobotStateSpace(robot)
        self.space_information = RobotSpaceInformation(
            self.state_space, endeffector, moved_object)
        # Attach the validity checker.
        validity_checker = RobotValidityChecker(
            self.space_information, collision_check_functions,
            constraint_functions)
        self.space_information.setStateValidityChecker(validity_checker)
        self.space_information.setup()
        super().__init__(self.space_information)

    def setOptimizationObjective(self, objectives: list) -> None:
        """
        Configures the optimization objective for the planner.

        Args:
            objectives (list): List of (objective, weight) tuples.
        """
        if len(objectives) == 1:
            self.optimization_objective = objectives[0][0](
                self.space_information)
        else:
            self.optimization_objective = RobotMultiOptimizationObjective(
                self.space_information, objectives)
        super().setOptimizationObjective(self.optimization_objective)

    def setPlanner(self, planner_type) -> None:
        """
        Configures the planner based on the given planner type.

        Args:
            planner_type: The planner class/type.
        """
        self.planner = planner_type(self.space_information)
        super().setPlanner(self.planner)

    def set_interpolation_precision(self, precision: float) -> None:
        """
        Sets the interpolation precision for path smoothing.

        Args:
            precision (float): Distance between interpolated states.
        """
        self.interpolation_precision = precision

    def setStartAndGoalStates(self, start: dict, goal: dict) -> None:
        """
        Sets the start and goal joint configurations for planning.

        Args:
            start (dict): Start joint configuration.
            goal (dict): Goal joint configuration.
        """
        start_list = self.state_space.dict_to_list(start)
        goal_list = self.state_space.dict_to_list(goal)
        start_state = self.space_information.list_to_state(start_list)
        goal_state = self.space_information.list_to_state(goal_list)
        super().setStartAndGoalStates(start_state, goal_state)

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = 5.0,
                        simplify: float = 1.0) -> tuple:
        """
        Plans a path from the start to goal configuration and returns the
        resulting JointPath if successful.

        Args:
            start (dict): Start joint configuration.
            goal (dict): Goal joint configuration.
            allowed_time (float): Maximum planning time in seconds.
            simplify (float): Factor for solution simplification.

        Returns:
            tuple: (solved (bool), JointPath or None)
        """
        self.clear()
        self.planner.clear()
        self.setStartAndGoalStates(start, goal)
        self.setup()

        solved = self.solve(allowed_time)
        joint_path = None

        if solved:
            if simplify:
                self.simplifySolution(simplify)
            sol_path = self.getSolutionPath()
            path_length = sol_path.length()
            interpolation_num = int(path_length /
                                    self.interpolation_precision)
            sol_path.interpolate(interpolation_num)
            states = sol_path.getStates()
            # Convert each state to a list of joint values.
            path_list = np.array([
                self.state_space.state_to_list(st) for st in states])
            print("Number of solution states: ", len(states))
            joint_path = JointPath(
                path_list.transpose(),
                tuple(self.state_space.joint_order)
            )
        else:
            print("No solution found")
        return solved, joint_path

    def clear(self) -> None:
        """
        Clears previous planning data.
        """
        super().clear()

    def setup(self) -> None:
        """
        Performs necessary setup before solving the planning problem.
        """
        super().setup()

    def solve(self, allowed_time: float = 5.0) -> bool:
        """
        Attempts to solve the planning problem within the given time.

        Args:
            allowed_time (float): Maximum time allowed for planning.

        Returns:
            bool: True if a solution is found, False otherwise.
        """
        return super().solve(allowed_time)

    def simplifySolution(self, factor: float = 1.0) -> None:
        """
        Simplifies the solution path using a factor.

        Args:
            factor (float): The simplification factor.
        """
        super().simplifySolution(factor)
