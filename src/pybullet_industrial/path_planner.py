import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)


class PbiObjectMover:
    """
    Objects to be relocated in the PyBullet simulation are managed.
    Each object is specified by its URDF and associated position and
    orientation offsets.
    """

    def __init__(self, urdf: list = None, position_offset: list = None,
                 orientation_offset: list = None):
        """
        The object mover is initialized with optional objects.

        Args:
            urdf (list, optional): List of URDF identifiers.
            position_offset (list, optional): List of 3D position offsets.
            orientation_offset (list, optional): List of quaternion offsets.
        """
        self.moving_objects = []
        if urdf is not None:
            # Each object is created and added with its offsets.
            for urdf_item, pos, ori in zip(urdf, position_offset,
                                           orientation_offset):
                self.add_object(urdf_item, pos, ori)

    def add_object(self, urdf, position_offset=None, orientation_offset=None):
        """
        An object is added by storing its URDF and offsets.

        Args:
            urdf: The URDF identifier for the object.
            position_offset (list or np.array, optional): 3D position offset.
                Defaults to [0, 0, 0].
            orientation_offset (list or np.array, optional): Quaternion offset
                [x, y, z, w]. Defaults to [0, 0, 0, 1].
        """
        if position_offset is None:
            position_offset = np.array([0, 0, 0])
        if orientation_offset is None:
            orientation_offset = np.array([0, 0, 0, 1])
        self.moving_objects.append((urdf, position_offset, orientation_offset))

    def match_moving_objects(self, position, orientation):
        """
        The base of each moving object is aligned with the robot's end
        effector.

        For every object, the new base pose is computed by multiplying the
        robot pose with the object's offset. The object's base is then
        updated in the simulation.

        Args:
            position (list or tuple): Current end effector position.
            orientation (list or tuple): Current end effector orientation.
        """
        for urdf, pos_off, ori_off in self.moving_objects:
            new_pos, new_ori = p.multiplyTransforms(
                position, orientation, pos_off, ori_off)
            p.resetBasePositionAndOrientation(urdf, new_pos, new_ori)


class PbiStateSpace(ob.RealVectorStateSpace):
    """
    An OMPL state space that represents the robot's joint configuration.
    The state space bounds are based on the robot's joint limits.

    Attributes:
        robot (RobotBase): The robot instance.
        joint_order (list): Ordered list of movable joints.
    """

    def __init__(self, robot: RobotBase) -> None:
        """
        The state space is initialized using the robot's joint limits.

        Args:
            robot (RobotBase): The robot instance with joint information.
        """
        self._robot = robot
        self._joint_order: list = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits()
        num_dims: int = len(self._joint_order)
        super().__init__(num_dims)
        self.setBounds(lower_limit, upper_limit)

    def list_to_state(self, joint_values: list) -> ob.State:
        """
        A list of joint values is converted to an OMPL state.

        Args:
            joint_values (list): Ordered list of joint values.

        Returns:
            ob.State: The corresponding OMPL state.
        """
        state = ob.State(self)
        for i, value in enumerate(joint_values):
            state[i] = value
        return state

    def state_to_list(self, state: ob.State) -> list:
        """
        An OMPL state is converted to a list of joint values.

        Args:
            state (ob.State): The OMPL state.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i in range(len(self._joint_order))]

    def dict_to_list(self, joint_dict: dict) -> list:
        """
        A dictionary of joint names and values is converted to an ordered
        list.

        Args:
            joint_dict (dict): Mapping from joint names to values.

        Returns:
            list: Ordered joint values as per self._joint_order.
        """
        return [joint_dict[joint] for joint in self._joint_order]

    def setBounds(self, lower_limit: list, upper_limit: list) -> None:
        """
        The state space bounds are set using the provided joint limits.

        Args:
            lower_limit (list): Lower limits for each joint.
            upper_limit (list): Upper limits for each joint.
        """
        bounds = ob.RealVectorBounds(self.getDimension())
        for i, joint in enumerate(self._joint_order):
            bounds.setLow(i, lower_limit[joint])
            bounds.setHigh(i, upper_limit[joint])
        super().setBounds(bounds)


class PbiSpaceInformation(ob.SpaceInformation):
    """
    OMPL SpaceInformation is extended with robot-specific functions.
    It handles state conversion and updates the robot configuration.

    Attributes:
        robot (RobotBase): The robot instance.
        state_space (PbiStateSpace): The custom state space.
        object_mover (PbiObjectMover): Optional mover for updating objects.
    """

    def __init__(self, state_space: PbiStateSpace,
                 object_mover: PbiObjectMover) -> None:
        """
        The space information is initialized with the state space and
        object mover.

        Args:
            state_space (PbiStateSpace): The robot's state space.
            object_mover (PbiObjectMover): The object mover instance.
        """
        super().__init__(state_space)
        self._robot = state_space._robot
        self._state_space: PbiStateSpace = state_space
        self._object_mover = object_mover

    def set_state(self, state: ob.State) -> None:
        """
        The robot configuration is updated based on the given state.
        Moving objects are updated if applicable.

        Args:
            state (ob.State): The state to be applied.
        """
        joint_positions = self._state_space.state_to_list(state)
        self._robot.reset_joint_position(
            dict(zip(self._state_space._joint_order, joint_positions)),
            True
        )
        if self._object_mover:
            position, orientation = self._robot.get_endeffector_pose()
            self._object_mover.match_moving_objects(position, orientation)


class PbiValidityChecker(ob.StateValidityChecker):
    """
    A state is validated by updating the robot configuration and performing
    collision and constraint tests.

    Attributes:
        si (PbiSpaceInformation): The robot's space information.
        collision_check_function (callable): Returns True if the state is
            collision free.
        constraint_function (callable): Returns True if constraints are met.
        clearance_function (callable): Returns a clearance value.
    """

    def __init__(self, si: PbiSpaceInformation,
                 collision_check_function,
                 constraint_function=None,
                 clearance_function=None) -> None:
        """
        The validity checker is initialized.

        Args:
            si (PbiSpaceInformation): The space information.
            collision_check_function (callable): Function to perform collision
                checks.
            constraint_function (callable, optional): Function to check
                additional constraints.
            clearance_function (callable, optional): Function returning a
                clearance value.
        """
        super().__init__(si)
        self._si = si
        self.collision_check_function = collision_check_function
        self.constraint_function = constraint_function
        self.clearance_function = clearance_function

    def isValid(self, state: ob.State) -> bool:
        """
        State validity is determined by applying clearance, constraints,
        and collision tests.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if the state is valid; False otherwise.
        """
        # Clearance is computed even if not used directly.
        self.clearance(state)
        if self.constraint_function:
            if not self.constraint_function():
                return False
        if not self.collision_check_function():
            return False
        return True

    def clearance(self, state: ob.State):
        """
        Clearance is computed for a state via collision tests.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields
            a higher cost.
        """
        self._si.set_state(state)
        if self.clearance_function is None:
            return None
        else:
            return self.clearance_function()


class PbiMultiOptimizationObjective(ob.MultiOptimizationObjective):
    """
    Multiple robot-specific optimization objectives are combined.

    Attributes:
        si (PbiSpaceInformation): The robot's space information.
    """

    def __init__(self, si: PbiSpaceInformation,
                 weighted_objective_list: list) -> None:
        """
        The multi-objective is initialized with weighted objectives.

        Args:
            si (PbiSpaceInformation): The robot's space information.
            weighted_objective_list (list): List of (objective, weight) pairs.
        """
        super().__init__(si)
        self._si = si
        # Each objective is added with its associated weight.
        for objective, weight in weighted_objective_list:
            self.addObjective(objective(self._si), weight)
        self.lock()


class PbiMaximizeMinClearanceObjective(ob.MaximizeMinClearanceObjective):
    """
    A cost objective is defined that rewards paths with higher obstacle
    clearance.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:
        """
        The clearance objective is initialized.

        Args:
            si (PbiSpaceInformation): The robot's space information.
        """
        super().__init__(si)
        self._si = si

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        The cost for a state is computed based on its minimum clearance.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields a higher
            cost.
        """
        return self._si.getStateValidityChecker().clearance(state)


class PbiClearanceObjective(ob.StateCostIntegralObjective):
    """
    The cost is computed based on state clearance. Lower cost is preferable.
    States with clearance below the target incur higher cost, while states
    above target incur lower cost until a cap is reached.
    """

    def __init__(self, si: PbiSpaceInformation,
                 importance: float = 0.5,
                 target_clearance: float = 1.0,
                 max_clearance: float = 2.0) -> None:
        """
        The clearance objective is initialized with provided parameters.

        Args:
            si (PbiSpaceInformation): The robot's space information.
            importance (float): Scaling factor in [0, 1] for cost assignment
                when clearance is below target.
            target_clearance (float): Ideal clearance with minimal cost.
            max_clearance (float): Clearance above which cost is capped.
        """
        super(PbiClearanceObjective, self).__init__(si, True)
        self._si = si
        self.importance = importance
        self.target_clearance = target_clearance
        self.max_clearance = max_clearance

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        The cost for a state is computed based on its clearance.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields a higher
            cost.
        """
        clearance = self._si.getStateValidityChecker().clearance(state)
        if clearance < 0:
            cost = self.importance
        elif clearance > self.max_clearance:
            cost = 1 - self.importance
        elif clearance < self.target_clearance:
            cost = (-(self.importance / self.target_clearance) * clearance +
                    self.importance)
        else:
            cost = ((1 - self.importance) /
                    (self.max_clearance - self.target_clearance)) * (
                        clearance - self.target_clearance)
        return ob.Cost(float(cost))


# Planner constants.
INTERPOLATION_PRECISION = 0.01
VALIDITY_RESOLUTION = 0.005


class PbiPlannerSimpleSetup(og.SimpleSetup):
    """
    All components are integrated to define and solve a robot planning
    problem. The state space, space information, validity checker,
    optimization objective, and planner are configured.
    """

    def __init__(self, robot: RobotBase,
                 collision_check_function: callable,
                 planner_type=None,
                 constraint_function=None,
                 clearance_function=None,
                 objective=None,
                 object_mover=None,
                 name: str = None) -> None:
        """
        Initializes the planning setup with robot-specific configs.

        Args:
            robot (RobotBase): The robot instance.
            collision_check_function (callable): Function to perform
                collision checks.
            planner_type (callable, optional): Callable that instantiates a
                planner given a PbiSpaceInformation instance.
            constraint_function (callable, optional): Function to check extra
                constraints.
            clearance_function (callable, optional): Function that returns a
                clearance value.
            objective (class, optional): Optimization objective class.
            object_mover (PbiObjectMover, optional): Mover for updating
                simulation objects.
            name (str, optional): Custom name for the setup.
        """
        # Store the robot instance.
        self._robot = robot

        # Initialize the state space using the robot's joint limits.
        self._state_space = PbiStateSpace(robot)

        # Create and assign the space information.
        super().__init__(PbiSpaceInformation(self._state_space, object_mover))
        self._si = self.getSpaceInformation()

        # Create and set the validity checker with collision,
        # constraint, and clearance functions.
        self._validity_checker = PbiValidityChecker(
            self._si,
            collision_check_function,
            constraint_function=constraint_function,
            clearance_function=clearance_function
        )
        self.setStateValidityChecker(self._validity_checker)

        # Configure the planner if a planner type is provided.
        if planner_type is not None:
            if callable(planner_type):
                self.setPlanner(planner_type)
            else:
                raise TypeError("planner_type must be callable")

        # Set the optimization objective if provided.
        if objective is not None:
            self.setOptimizationObjective(objective)

        # Set interpolation precision and validity resolution.
        self.set_interpolation_precision(INTERPOLATION_PRECISION)
        self._si.setStateValidityCheckingResolution(VALIDITY_RESOLUTION)

        # Assign a custom name or a default name.
        self.name = name if name is not None else "PbiPlannerSimpleSetup"

    def set_constraint_function(self, constraint_function: callable) -> None:
        """
        Updates the validity checker's constraint function.

        Args:
            constraint_function: Callable used to check additional constraints.
        """
        self._validity_checker.constraint_function = constraint_function

    def set_clearance_function(self, clearance_function: callable) -> None:
        """
        Assigns the clearance function for state evaluation.

        Args:
            clearance_function: Callable that returns a clearance value.
        """
        self._validity_checker.clearance_function = clearance_function

    def set_collision_check_function(
            self,
            collision_check_function: callable) -> None:
        """
        Sets the collision check function for validity tests.

        Args:
            collision_check_function: Callable that verifies collision-free
                states.
        """
        self._validity_checker.collision_check_function = (
            collision_check_function
        )

    def setOptimizationObjective(self, objective: callable) -> None:
        """
        The optimization objective for the planner is set.

        Args:
            objective (class or None): The optimization objective class.
        """
        if objective is not None:
            super().setOptimizationObjective(objective(self._si))

    def setPlanner(self, planner_type: callable) -> None:
        """
        The planner is configured.

        Args:
            planner_type: The planner class/type to be instantiated.
        """
        super().setPlanner(planner_type(self._si))

    def set_interpolation_precision(self, precision: float) -> None:
        """
        The interpolation precision is set for path generation.

        Args:
            precision (float): Distance between interpolated states.
        """
        self._interpolation_precision = precision

    def setStartAndGoalStates(self, start: dict, goal: dict) -> None:
        """
        Defines start and goal states for planning.

        Args:
            start (dict): Starting joint configuration.
            goal (dict): Goal joint configuration.
        """
        # Convert start configuration from dictionary to ordered list.
        start_list = self._state_space.dict_to_list(start)
        # Convert goal configuration from dictionary to ordered list.
        goal_list = self._state_space.dict_to_list(goal)
        # Convert the ordered list to an OMPL state for start.
        start_state = self._state_space.list_to_state(start_list)
        # Convert the ordered list to an OMPL state for goal.
        goal_state = self._state_space.list_to_state(goal_list)
        # Define start and goal states in the parent class.
        super().setStartAndGoalStates(start_state, goal_state)

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = 5.0,
                        simplify: bool = True) -> tuple:
        """
        Plans a path from start to goal configuration and returns the result.

        Args:
            start (dict): Starting joint configuration.
            goal (dict): Goal joint configuration.
            allowed_time (float): Maximum planning time in seconds.
            simplify (float): Factor for solution simplification

        Returns:
            tuple: (solved (bool), JointPath or None) indicating whether
                a valid path was found and the corresponding joint path.
        """
        # Clear previous planning data.
        self.clear()
        if self.getPlanner() is not None:
            self.getPlanner().clear()

        # Define start and goal states.
        self.setStartAndGoalStates(start, goal)
        self.setup()

        # Attempt to solve the planning problem.
        solved = self.solve(allowed_time)
        joint_path = None

        # Process solution if a valid path is found.
        if solved:
            # Simplify the solution if requested.
            if simplify:
                self.simplifySolution(True)

            # Retrieve and process the solution path.
            sol_path = self.getSolutionPath()
            path_length = sol_path.length()
            interpolation_num = int(
                path_length / self._interpolation_precision)
            sol_path.interpolate(interpolation_num)

            # Convert states to a joint configuration path.
            states = sol_path.getStates()
            path_list = np.array([self._state_space.state_to_list(st)
                                  for st in states])
            joint_path = JointPath(path_list.transpose(),
                                   tuple(self._state_space._joint_order))
        else:
            # Inform that no valid solution was found.
            print("No solution found")

        # Return the planning outcome and joint path.
        return solved, joint_path
