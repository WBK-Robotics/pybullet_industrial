import sys
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import CollisionChecker
from pybullet_industrial import RobotBase
from pybullet_industrial import JointPath
import numpy as np
import pybullet as p

INTERPOLATE_NUM = 500  # Number of segments for interpolating the solution path
DEFAULT_PLANNING_TIME = 5.0  # Default maximum allowed time for planning

class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))

class ValidityChecker(ob.StateValidityChecker):
    """
    A class for checking the validity of states in the planning space.

    Args:
        si: The space information object for the planning problem.
        robot: Instance of RobotBase representing the robot model and its
               kinematics.
        collision_checker: Instance of CollisionChecker for collision
                           detection management.
        joint_order: List of joint names to include in the planning space.
    """

    def __init__(self, space_information, robot: RobotBase, collision_checker: CollisionChecker,
                 joint_order: list):
        super(ValidityChecker, self).__init__(space_information)
        self.robot = robot
        self.collision_checker = collision_checker
        self.joint_order = joint_order

    def isValid(self, state):
        """
        Checks if a given state is valid by ensuring no collisions with the
        environment or self-collisions.

        Args:
            state: The state of the robot (list of joint positions).

        Returns:
            bool: True if the state is valid (collision-free), False otherwise.
        """
        joint_positions = []
        for i, joint_name in enumerate(self.joint_order):
            joint_positions.append(state[i])
        self.robot.reset_joint_position(dict(zip(self.joint_order, joint_positions)), True)

        # Check for collisions
        if not self.collision_checker.check_collision():
            return False

        # Check if end-effector is upright
        if not self.check_endeffector_upright():
            return False

        return True

    def check_endeffector_upright(self):
        """
        Constraint function
        """
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        # Target orientation
        target_orientation = np.array([-np.pi / 2, 0, 0])

        # Tolerance of ±0.1
        tolerance = np.array([0.3, 0.3, 2*np.pi])

        # Current orientation
        if np.all(np.abs(orientation - target_orientation) <= tolerance):
            return True
        else:
            return False


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
                 planner_name="BITstar", selected_joint_names: set = None, objective="PathLength"):
        self.robot = robot
        self.collision_checker = collision_checker

        self.joint_order = robot.get_moveable_joints(selected_joint_names)[0]
        self.real_vector = True
        if any(value == np.inf for value in robot.get_joint_limits(self.joint_order)[1].values()):
            self.real_vector = False
            self.space = self.build_compound_space()
        else:
            self.space = self.build_realvector_space()

        # Set space information
        self.space_information = ob.SpaceInformation(self.space)
        self.validity_checker = ValidityChecker(self.space_information, robot, collision_checker, self.joint_order)
        self.space_information.setStateValidityChecker(self.validity_checker)
        self.space_information.setup()

        self.problem_definition = ob.ProblemDefinition(self.space_information)

        # Default Path lenght for now
        self.problem_definition.setOptimizationObjective(self.allocateObjective(objective))

        self.planner = self.allocatePlanner(planner_name)


    def build_realvector_space(self):
        # Configure bounds for the state space
        number_of_dimensions = len(self.joint_order)
        bounds = ob.RealVectorBounds(number_of_dimensions)
        lower_limit, upper_limit = self.robot.get_joint_limits(set(self.joint_order))

        for i, joint_name in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint_name]-.1)
            bounds.setHigh(i, upper_limit[joint_name]+.1)

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
        if state is not None:
            target = {}
            if not self.real_vector:
                joint_positions = self.coumpound_state_to_list(state)
            else:
                joint_positions = self.realvector_state_to_list(state)

            for joint_name, joint_position in zip(self.joint_order, joint_positions):
                target[joint_name] = joint_position
            self.robot.reset_joint_position(target, True)

        # Check for collisions
        if not self.collision_checker.check_collision():
            return False

        # Check if end-effector is upright
        if not self.check_endeffector_upright():
            return False

        return True

    def check_endeffector_upright(self):
        """
        Constraint function
        """
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        # Target orientation
        target_orientation = np.array([-np.pi / 2, 0, 0])

        # Tolerance of ±0.1
        tolerance = np.array([0.3, 0.3, 2*np.pi])

        # Current orientation
        if np.all(np.abs(orientation - target_orientation) <= tolerance):
            return True
        else:
            return False

    # Keep these in alphabetical order and all lower case
    def allocateObjective(self, objectiveType):
        if objectiveType.lower() == "pathclearance":
            return ClearanceObjective(self.space_information)
        elif objectiveType.lower() == "pathlength":
            return ob.PathLengthOptimizationObjective(self.space_information)
        elif objectiveType.lower() == "thresholdpathlength":
                obj = ob.PathLengthOptimizationObjective(self.space_information)
                obj.setCostThreshold(ob.Cost(1.51))
                return obj

        elif objectiveType.lower() == "weightedlengthandclearancecombo":
                lengthObj = ob.PathLengthOptimizationObjective(self.space_information)
                clearObj = ClearanceObjective(self.space_information)
                opt = ob.MultiOptimizationObjective(self.space_information)
                opt.addObjective(lengthObj, 5.0)
                opt.addObjective(clearObj, 1.0)
                return opt
        else:
            ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


    # Keep these in alphabetical order and all lower case
    def allocatePlanner(self, plannerType):
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
            ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")

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
        self.problem_definition.setStartAndGoalStates(s, g)

        self.planner.setProblemDefinition(self.problem_definition)
        self.planner.setup()

        # Solve the planning problem
        solved = self.planner.solve(allowed_time)
        res = False
        joint_path = None
        if solved:
            sol_path_geometric = self.problem_definition.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            if self.real_vector:
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
        self.robot.reset_joint_position(orig_robot_state, True)
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