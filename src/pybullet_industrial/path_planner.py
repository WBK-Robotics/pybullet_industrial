import sys
import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
from pybullet_industrial import (RobotBase, JointPath)


class PbiObjectMover:
    """
    Manages objects to be moved in the PyBullet simulation.

    Each object is defined by its URDF and associated position and
    orientation offsets.
    """

    def __init__(self, urdf: list = None, position_offset: list = None,
                 orientation_offset: list = None):
        """
        Initializes the mover with optional objects.

        Args:
            urdf (list, optional): List of URDF identifiers.
            position_offset (list, optional): List of 3D position offsets.
            orientation_offset (list, optional): List of quaternion offsets.
        """
        self.moving_objects = []
        if urdf is not None:
            # Create and add each object with its corresponding offsets.
            for urdf_item, pos, ori in zip(urdf, position_offset,
                                           orientation_offset):
                self.add_object(urdf_item, pos, ori)

    def add_object(self, urdf, position_offset=None, orientation_offset=None):
        """
        Adds an object to be moved by storing its URDF and offsets.

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
        Aligns each moving object's base with the robot's end effector.

        For every registered object, the new base pose is computed by
        multiplying the robot's pose with the object's offset. The object's
        base is then updated in the simulation.

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
    An OMPL state space representing the robot's joint configuration.

    The bounds are set based on the robot's joint limits.

    Attributes:
        robot (RobotBase): The robot instance.
        joint_order (list): The ordered list of movable joints.
    """

    def __init__(self, robot: RobotBase) -> None:
        """
        Initializes the state space using the robot's joint limits.

        Args:
            robot (RobotBase): The robot instance with joint info.
        """
        self.robot: RobotBase = robot
        # Get the list of movable joints (first element returned).
        self.joint_order: list = robot.get_moveable_joints()[0]
        lower_limit, upper_limit = robot.get_joint_limits(
            set(self.joint_order))
        num_dims: int = len(self.joint_order)
        # Initialize the underlying RealVectorStateSpace.
        super().__init__(num_dims)
        self.setBounds(lower_limit, upper_limit)

    def list_to_state(self, joint_values: list) -> ob.State:
        """
        Converts a list of joint values to an OMPL state.

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
        Converts an OMPL state to a list of joint values.

        Args:
            state (ob.State): The OMPL state.

        Returns:
            list: A list of joint values.
        """
        return [state[i] for i in range(len(self.joint_order))]

    def dict_to_list(self, joint_dict: dict) -> list:
        """
        Converts a dictionary of joint names/values to an ordered list.

        Args:
            joint_dict (dict): Mapping from joint names to values.

        Returns:
            list: Ordered joint values according to self.joint_order.
        """
        return [joint_dict[joint] for joint in self.joint_order]

    def setBounds(self, lower_limit: list, upper_limit: list) -> None:
        """
        Sets the state space bounds using the provided joint limits.

        Args:
            lower_limit (list): Lower limits for each joint.
            upper_limit (list): Upper limits for each joint.
        """
        bounds = ob.RealVectorBounds(self.getDimension())
        for i, joint in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint])
            bounds.setHigh(i, upper_limit[joint])
        super().setBounds(bounds)


class PbiSpaceInformation(ob.SpaceInformation):
    """
    Extends OMPL's SpaceInformation with robot-specific functionality.

    It handles state conversion and updates the robot configuration.

    Attributes:
        robot (RobotBase): The robot instance.
        state_space (PbiStateSpace): The custom state space.
        object_mover (PbiObjectMover): Optional mover for updating objects.
    """

    def __init__(self, state_space: PbiStateSpace,
                 object_mover: PbiObjectMover,
                 validity_resolution: float = 0.005) -> None:
        """
        Initializes the space information with the state space and
        resolution.

        Args:
            state_space (PbiStateSpace): The robot's state space.
            object_mover (PbiObjectMover): The object mover instance.
            validity_resolution (float): Resolution for state validity checking.
        """
        super().__init__(state_space)
        self.setStateValidityCheckingResolution(validity_resolution)
        self.robot: RobotBase = state_space.robot
        self.state_space: PbiStateSpace = state_space
        self.object_mover = object_mover

    def set_state(self, state: ob.State) -> None:
        """
        Updates the robot's configuration based on the given state and
        updates moving objects if applicable.

        Args:
            state (ob.State): The state to be applied.
        """
        joint_positions = self.state_space.state_to_list(state)
        self.robot.reset_joint_position(
            dict(zip(self.state_space.joint_order, joint_positions)),
            True
        )
        if self.object_mover:
            position, orientation = self.robot.get_endeffector_pose()
            self.object_mover.match_moving_objects(position, orientation)


class PbiValidityChecker(ob.StateValidityChecker):
    """
    Validates a state by updating the robot's configuration and running
    collision and constraint tests.

    Attributes:
        space_information (PbiSpaceInformation): The robot's space info.
        collision_check_function (callable): Function returning True if the
            state is collision free.
        constraint_function (callable): Function returning True if constraints
            are satisfied.
        clearance_function (callable): Function returning a clearance value.
    """

    def __init__(self, space_information: PbiSpaceInformation,
                 collision_check_function,
                 constraint_function=None,
                 clearance_function=None) -> None:
        """
        Initializes the validity checker.

        Args:
            space_information (PbiSpaceInformation): The space info.
            collision_check_function (callable): Function to perform collision
                checks.
            constraint_function (callable, optional): Function to check
                additional constraints.
            clearance_function (callable, optional): Function that returns
                the clearance value.
        """
        super().__init__(space_information)
        self.space_information: PbiSpaceInformation = space_information
        self.collision_check_function = collision_check_function
        self.constraint_function = constraint_function
        self.clearance_function = clearance_function

    def isValid(self, state: ob.State) -> bool:
        """
        Checks if a given state is valid by applying clearance,
        constraints, and collision tests.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if the state is valid; False otherwise.
        """
        # Compute clearance (even if not used directly for validity)
        self.clearance(state)
        if self.constraint_function:
            if not self.constraint_function():
                return False
        if not self.collision_check_function():
            return False
        return True

    def clearance(self, state: ob.State):
        """
        Computes the clearance of a state by applying collision tests.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields
            a higher cost.
        """
        self.space_information.set_state(state)
        if self.clearance_function is None:
            return ob.Cost(0)
        else:
            return ob.Cost(self.clearance_function())


class PbiMultiOptimizationObjective(ob.MultiOptimizationObjective):
    """
    Combines multiple robot-specific optimization objectives.

    Attributes:
        si (PbiSpaceInformation): The robot's space information.
    """

    def __init__(self, si: PbiSpaceInformation,
                 weighted_objective_list: list) -> None:
        """
        Initializes the multi-objective with weighted objectives.

        Args:
            si (PbiSpaceInformation): The robot's space information.
            weighted_objective_list (list): List of (objective, weight) pairs.
        """
        super().__init__(si)
        self.si: PbiSpaceInformation = si
        # Add each objective with its associated weight.
        for objective, weight in weighted_objective_list:
            self.addObjective(objective(self.si), weight)
        self.lock()


class PbiPathClearanceObjective(ob.MaximizeMinClearanceObjective):
    """
    A cost objective that rewards paths with higher clearance from
    obstacles.

    The cost increases when the robot is too close to obstacles.
    """

    def __init__(self, si: PbiSpaceInformation) -> None:
        """
        Initializes the clearance objective.

        Args:
            si (PbiSpaceInformation): The robot's space information.
        """
        super().__init__(si)
        self.si: PbiSpaceInformation = si

    def stateCost(self, state: ob.State) -> ob.Cost:
        """
        Computes the cost for a state based on its minimum clearance.

        Args:
            state (ob.State): The state to evaluate.

        Returns:
            ob.Cost: The computed cost, where lower clearance yields
            a higher cost.
        """
        return self.si.getStateValidityChecker().clearance(state)


class PbiPlannerSimpleSetup(og.SimpleSetup):
    """
    Integrates all components to define and solve a robot planning
    problem.

    Sets up the state space, space information, validity checker,
    optimization objective, and planner. Provides an API to plan a
    path between start and goal joint configurations.
    """

    def __init__(self, robot: RobotBase,
                 collision_check_function,
                 planner_type,
                 interpolation_precision: float = 0.01,
                 constraint_function=None,
                 clearance_function=None,
                 objective=None,
                 object_mover=None,
                 name: str = None) -> None:
        """
        Initializes the planning setup.

        Args:
            robot (RobotBase): The robot instance.
            collision_check_function (callable): Function to perform collision
                checks.
            planner_type: The planner class/type.
            interpolation_precision (float): Precision for path interpolation.
            constraint_function (callable, optional): Function to check
                additional constraints.
            clearance_function (callable, optional): Function that returns a
                clearance value.
            objective (class, optional): Optimization objective class.
            object_mover (PbiObjectMover, optional): Mover for updating objects.
            name (str, optional): Custom name for the setup.
        """
        self.name = name if name is not None else "PbiPlannerSimpleSetup"
        self.set_interpolation_precision(interpolation_precision)
        self.robot = robot
        self.state_space = PbiStateSpace(robot)
        self.space_information = PbiSpaceInformation(
            self.state_space, object_mover)
        self.validity_checker = PbiValidityChecker(
            self.space_information, collision_check_function,
            clearance_function=clearance_function)
        self.planner_type = planner_type
        self.objective = objective
        self.update_constraints(constraint_function)

    def update_constraints(self, constraint_function) -> None:
        """
        Activates the constraint function and reinitializes the planner
        setup.

        Args:
            constraint_function (callable): Function to check additional
                constraints.
        """
        self.validity_checker.constraint_function = constraint_function
        self.space_information.setStateValidityChecker(
            self.validity_checker)
        self.space_information.setup()
        # Reinitialize the SimpleSetup with the updated space info.
        super().__init__(self.space_information)
        self.setPlanner(self.planner_type)
        self.setOptimizationObjective(self.objective)

    def setOptimizationObjective(self, objective) -> None:
        """
        Sets the optimization objective for the planner.

        Args:
            objective (class or None): The optimization objective class to set.
        """
        self.objective = objective
        if objective is not None:
            super().setOptimizationObjective(
                objective(self.space_information))
        else:
            super().setOptimizationObjective(None)

    def setPlanner(self, planner_type) -> None:
        """
        Configures the planner.

        Args:
            planner_type: The planner class/type to instantiate.
        """
        self.planner = planner_type(self.space_information)
        super().setPlanner(self.planner)

    def set_interpolation_precision(self, precision: float) -> None:
        """
        Sets the precision used during path interpolation.

        Args:
            precision (float): Distance between interpolated states.
        """
        self.interpolation_precision = precision

    def setStartAndGoalStates(self, start: dict, goal: dict) -> None:
        """
        Defines the start and goal states for planning.

        Args:
            start (dict): The starting joint configuration.
            goal (dict): The goal joint configuration.
        """
        start_list = self.state_space.dict_to_list(start)
        goal_list = self.state_space.dict_to_list(goal)
        start_state = self.state_space.list_to_state(start_list)
        goal_state = self.state_space.list_to_state(goal_list)
        super().setStartAndGoalStates(start_state, goal_state)

    def plan_start_goal(self, start: dict, goal: dict,
                        allowed_time: float = 5.0,
                        simplify: float = 1.0) -> tuple:
        """
        Plans a path from the start to goal configuration and returns the result.

        Args:
            start (dict): Starting joint configuration.
            goal (dict): Goal joint configuration.
            allowed_time (float): Maximum planning time in seconds.
            simplify (float): Factor for solution simplification (non-zero
                triggers path smoothing).

        Returns:
            tuple: (solved (bool), JointPath or None) depending on whether a
            valid path was found.
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
            interpolation_num = int(path_length / self.interpolation_precision)
            sol_path.interpolate(interpolation_num)
            states = sol_path.getStates()
            path_list = np.array([self.state_space.state_to_list(st)
                                  for st in states])
            print("Number of solution states: ", len(states))
            joint_path = JointPath(path_list.transpose(),
                                   tuple(self.state_space.joint_order))
        else:
            print("No solution found")
        return solved, joint_path
