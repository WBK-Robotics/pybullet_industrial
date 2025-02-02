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

###############################################################################
# SamplingSpace Class: Handles state space creation and state conversion.
###############################################################################
class SamplingSpace:
    def __init__(self, robot: RobotBase, joint_order: list):
        """
        Creates a sampling space based on the robot's joint limits.

        Args:
            robot: An instance of RobotBase.
            joint_order: A list of joint names to include in the space.
        """
        self.robot = robot
        self.joint_order = joint_order

        # Retrieve joint limits (expected as two dicts: lower and upper limits)
        lower_limit, upper_limit = self.robot.get_joint_limits(set(self.joint_order))

        # If any joint has an infinite limit, use a compound space; otherwise, use a real vector space.
        if any(np.isinf(lower_limit[joint]) or np.isinf(upper_limit[joint]) for joint in self.joint_order):
            self.real_vector = False
            self.space = self.build_compound_space(lower_limit, upper_limit)
        else:
            self.real_vector = True
            self.space = self.build_realvector_space(lower_limit, upper_limit)

    def build_realvector_space(self, lower_limit, upper_limit):
        """Builds a RealVectorStateSpace with a small margin on the joint limits."""
        number_of_dimensions = len(self.joint_order)
        bounds = ob.RealVectorBounds(number_of_dimensions)
        for i, joint_name in enumerate(self.joint_order):
            bounds.setLow(i, lower_limit[joint_name] - 0.1)
            bounds.setHigh(i, upper_limit[joint_name] + 0.1)
        space = ob.RealVectorStateSpace(number_of_dimensions)
        space.setBounds(bounds)
        return space

    def build_compound_space(self, lower_limit, upper_limit):
        """Builds a CompoundStateSpace that can handle different joint types."""
        space = ob.CompoundStateSpace()
        for joint_name in self.joint_order:
            if lower_limit[joint_name] == -np.inf and upper_limit[joint_name] == np.inf:
                # For continuous joints (e.g., unbounded rotations) we use SO2
                space.addSubspace(ob.SO2StateSpace(), 1.0)
            else:
                subspace = ob.RealVectorStateSpace(1)
                subspace.setBounds(lower_limit[joint_name], upper_limit[joint_name])
                space.addSubspace(subspace, 1.0)
        return space

    def get_space(self):
        """Returns the constructed state space."""
        return self.space

    def compound_state_to_list(self, state):
        """
        Converts a compound OMPL state object to a list of joint values.
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
        Converts a RealVector OMPL state object to a list of joint values.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]

###############################################################################
# Other Classes remain largely the same, with minor modifications.
###############################################################################
class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))

class ValidityChecker(ob.StateValidityChecker):
    """
    Checks whether a given state is collision-free and meets an additional
    end-effector orientation constraint.
    """
    def __init__(self, space_information, robot: RobotBase, collision_checker: CollisionChecker,
                 joint_order: list):
        super(ValidityChecker, self).__init__(space_information)
        self.robot = robot
        self.collision_checker = collision_checker
        self.joint_order = joint_order

    def isValid(self, state):
        # Convert the state to joint positions (assuming state is indexable)
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        self.robot.reset_joint_position(dict(zip(self.joint_order, joint_positions)), True)

        if not self.collision_checker.check_collision():
            return False

        if not self.check_endeffector_upright():
            return False

        return True

    def check_endeffector_upright(self):
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        target_orientation = np.array([-np.pi / 2, 0, 0])
        tolerance = np.array([0.3, 0.3, 2 * np.pi])
        return np.all(np.abs(orientation - target_orientation) <= tolerance)

class PathPlanner:
    """
    Sets up and manages motion planning for a robot.
    """
    def __init__(self, robot: RobotBase, collision_checker: CollisionChecker,
                 planner_name="BITstar", selected_joint_names: set = None, objective="PathLength"):
        self.robot = robot
        self.collision_checker = collision_checker

        # Get the joint order from the robot (using specified joints or all moveable joints)
        self.joint_order = robot.get_moveable_joints(selected_joint_names)[0]

        # Use SamplingSpace to build the state space and conversion methods
        self.sampling_space = SamplingSpace(robot, self.joint_order)
        self.space = self.sampling_space.get_space()
        self.real_vector = self.sampling_space.real_vector

        # Set up the OMPL space information and validity checker
        self.space_information = ob.SpaceInformation(self.space)
        self.validity_checker = ValidityChecker(self.space_information, robot, collision_checker, self.joint_order)
        self.space_information.setStateValidityChecker(self.validity_checker)
        self.space_information.setup()

        self.problem_definition = ob.ProblemDefinition(self.space_information)
        self.problem_definition.setOptimizationObjective(self.allocateObjective(objective))

        self.planner = self.allocatePlanner(planner_name)

    def is_state_valid(self, state):
        if state is not None:
            target = {}
            if not self.real_vector:
                joint_positions = self.sampling_space.compound_state_to_list(state)
            else:
                joint_positions = self.sampling_space.realvector_state_to_list(state)
            for joint_name, joint_position in zip(self.joint_order, joint_positions):
                target[joint_name] = joint_position
            self.robot.reset_joint_position(target, True)

        if not self.collision_checker.check_collision():
            return False

        if not self.check_endeffector_upright():
            return False

        return True

    def check_endeffector_upright(self):
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        target_orientation = np.array([-np.pi / 2, 0, 0])
        tolerance = np.array([0.3, 0.3, 2 * np.pi])
        return np.all(np.abs(orientation - target_orientation) <= tolerance)

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
        orig_robot_state = start
        s, g = self.set_start_goal_states(start, goal)
        self.problem_definition.setStartAndGoalStates(s, g)

        self.planner.setProblemDefinition(self.problem_definition)
        self.planner.setup()

        solved = self.planner.solve(allowed_time)
        res = False
        joint_path = None
        if solved:
            sol_path_geometric = self.problem_definition.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            if self.real_vector:
                sol_path_list = np.array(
                    [self.sampling_space.realvector_state_to_list(state) for state in sol_path_states])
            else:
                sol_path_list = np.array(
                    [self.sampling_space.compound_state_to_list(state) for state in sol_path_states])
            joint_path = JointPath(sol_path_list.transpose(), self.joint_order)
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_robot_state, True)
        return res, joint_path

    def set_start_goal_states(self, start, goal):
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i, joint_name in enumerate(self.joint_order):
            s[i] = start[joint_name]
            g[i] = goal[joint_name]
        return s, g