import sys
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from pybullet_industrial import (CollisionChecker, RobotBase, JointPath)
import numpy as np
import pybullet as p

# Global constant for continuous joint bounds.
CONTINIOUS_JOINT_BOUNDS = 3 * np.pi

INTERPOLATE_NUM = 500  # Number of segments for interpolating the path
DEFAULT_PLANNING_TIME = 5.0  # Maximum planning time in seconds


class SamplingSpace:
    """Creates an OMPL state space based on robot joint limits and provides
    state conversion and state creation utilities.

    This class always creates a RealVectorStateSpace. If a joint is
    continuous (i.e. has an infinite limit), its bounds are set to [-CONTINIOUS_JOINT_BOUNDS, CONTINIOUS_JOINT_BOUNDS].
    Otherwise, a small margin is added to the joint limits.

    It also includes a helper method to set the start and goal states.

    Args:
        robot (RobotBase): The robot model and its kinematics.
        joint_order (list): List of joint names for the state space.
    """
    def __init__(self, robot: RobotBase, joint_order: list):
        # Retrieve joint limits as two dictionaries (lower and upper bounds)
        lower_limit, upper_limit = robot.get_joint_limits(set(joint_order))
        self.joint_order = joint_order
        self.robot = robot

        # Always use a RealVectorStateSpace.
        self.real_vector = True
        self.space = self.build_realvector_space(lower_limit, upper_limit)

    def build_realvector_space(self, lower_limit: dict, upper_limit: dict):
        """Builds a RealVectorStateSpace with special handling for
        continuous joints.

        For each joint:
          - If a joint limit is infinite, set the bounds to [-CONTINIOUS_JOINT_BOUNDS, CONTINIOUS_JOINT_BOUNDS].
          - Otherwise, use the provided limit with a small margin (±0.1).

        Args:
            lower_limit (dict): Lower limits for each joint.
            upper_limit (dict): Upper limits for each joint.

        Returns:
            ob.RealVectorStateSpace: The constructed real vector state space.
        """
        num_dims = len(self.joint_order)
        bounds = ob.RealVectorBounds(num_dims)
        for i, joint in enumerate(self.joint_order):
            if np.isinf(lower_limit[joint]) or np.isinf(upper_limit[joint]):
                bounds.setLow(i, -CONTINIOUS_JOINT_BOUNDS)
                bounds.setHigh(i, CONTINIOUS_JOINT_BOUNDS)
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
            list: Joint values extracted from the state.
        """
        return [state[i] for i, _ in enumerate(self.joint_order)]

    def get_space(self):
        """Returns the constructed OMPL state space.

        Returns:
            ob.StateSpace: The state space used for planning.
        """
        return self.space

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
    """An objective that maximizes clearance from obstacles by penalizing low
    clearance states.

    The cost is computed as the reciprocal of the state's clearance plus a
    small constant, encouraging paths that maintain a safe distance from
    obstacles.

    Args:
        si (ob.SpaceInformation): The space information for the planning
            problem.
    """
    def __init__(self, si: ob.SpaceInformation):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    def stateCost(self, s: ob.State):
        """Computes the cost of a state based on its clearance.

        Args:
            s (ob.State): The state for which to compute the cost.

        Returns:
            ob.Cost: The computed cost.
        """
        # Uses the clearance computed by the ValidityChecker, which now
        # relies on the refactored collision checker.
        return ob.Cost(
            1 / (self.si_.getStateValidityChecker().clearance(s) +
                 sys.float_info.min)
        )


class ValidityChecker(ob.StateValidityChecker):
    """Checks if a state is collision-free and meets end-effector constraints.

    The checker resets the robot to a given state, verifies collisions, and
    ensures the end-effector is correctly oriented.

    Args:
        space_information (ob.SpaceInformation): The state space information.
        robot (RobotBase): The robot instance.
        collision_checker_list (list): List of CollisionChecker objects to check for collisions.
        joint_order (list): List of joint names corresponding to the state.
    """
    def __init__(self, space_information: ob.SpaceInformation, robot: RobotBase,
                 collision_checker_list: list, joint_order: list):
        super(ValidityChecker, self).__init__(space_information)
        self.robot = robot
        self.collision_checker_list = collision_checker_list
        self.joint_order = joint_order

    def clearance(self, state: ob.State):
        """Computes the clearance of a given state using the refactored
        collision checkers.

        The robot is reset to the state provided and each collision checker in
        the list is used to obtain a clearance. The minimum clearance is returned.

        Args:
            state (ob.State): The state for which to compute clearance.

        Returns:
            float: The clearance value.
        """
        # Extract joint positions and reset the robot.
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        self.robot.reset_joint_position(dict(zip(self.joint_order, joint_positions)), True)
        # Gather clearances from all collision checkers; the overall clearance is the minimum.
        clearances = [cc.get_max_distance() for cc in self.collision_checker_list]
        return min(clearances)

    def isValid(self, state: ob.State):
        """Checks if the provided state is valid.

        A state is valid if it is collision-free and the end-effector is
        correctly oriented.

        Args:
            state (ob.State): The state to validate.

        Returns:
            bool: True if valid, False otherwise.
        """
        # Extract joint positions from the state.
        joint_positions = [state[i] for i, _ in enumerate(self.joint_order)]
        self.robot.reset_joint_position(
            dict(zip(self.joint_order, joint_positions)), True
        )

        for collision_checker in self.collision_checker_list:
            if not collision_checker.check_collision():
                return False

        # Uncomment the following lines if end-effector orientation checking is needed.
        # if not self.check_endeffector_upright():
        #     return False

        return True

    def check_endeffector_upright(self):
        """Checks if the robot's end-effector is upright within tolerance.

        Returns:
            bool: True if upright, False otherwise.
        """
        orientation = p.getEulerFromQuaternion(
            self.robot.get_endeffector_pose()[1]
        )
        target = np.array([-np.pi / 2, 0, 0])
        tol = np.array([0.3, 0.3, 2 * np.pi])
        return np.all(np.abs(orientation - target) <= tol)


class PathPlanner:
    """Sets up and executes motion planning for a robot in a constrained
    environment.

    This class configures the state space, sets up the planning problem, and
    provides methods to solve for a path between start and goal configurations.

    Args:
        robot (RobotBase): The robot instance for which to plan.
        collision_checker_list (list): List of CollisionChecker objects to check for collisions.
        planner_name (str, optional): The planner to use (e.g., "BITstar").
            Defaults to "BITstar".
        selected_joint_names (set, optional): Set of joint names to include.
            Defaults to None (all moveable joints are used).
        objective (str, optional): The optimization objective (e.g.,
            "PathLength"). Defaults to "PathLength".
    """
    def __init__(self, robot: RobotBase, collision_checker_list: list,
                 planner_name: str = "BITstar", selected_joint_names: set = None,
                 objective: str = "PathLength"):
        self.robot = robot
        self.collision_checker_list = collision_checker_list

        # Determine the joint order from the robot.
        self.joint_order = robot.get_moveable_joints(selected_joint_names)[0]

        # Create the state space using SamplingSpace.
        self.sampling_space = SamplingSpace(robot, self.joint_order)
        self.space = self.sampling_space.get_space()
        self.real_vector = self.sampling_space.real_vector

        # Set up OMPL space information and validity checking.
        self.space_information = ob.SpaceInformation(self.space)
        self.validity_checker = ValidityChecker(self.space_information, robot,
                                                collision_checker_list,
                                                self.joint_order)
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
        """Selects and allocates an optimization objective for the planning
        problem.

        Args:
            objectiveType (str): The type of optimization objective.

        Returns:
            ob.OptimizationObjective: The allocated objective.
        """
        if objectiveType.lower() == "pathclearance":
            return ClearanceObjective(self.space_information)
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
            clear_obj = ClearanceObjective(self.space_information)
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

        This method sets up the start and goal states, runs the planner, and
        returns whether a solution was found along with the resulting joint
        path.

        Args:
            start (dict): Joint values for the start configuration.
            goal (dict): Joint values for the goal configuration.
            allowed_time (float, optional): Maximum planning time in seconds.
                Defaults to DEFAULT_PLANNING_TIME.

        Returns:
            tuple(bool, JointPath): A tuple containing a success flag and the
            joint path.
        """
        orig_state = start
        # Use the SamplingSpace helper to create start and goal states.
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
                path_list.transpose(), tuple(self.joint_order)
            )
            res = True
        else:
            print("No solution found")

        self.robot.reset_joint_position(orig_state, True)
        return res, joint_path
