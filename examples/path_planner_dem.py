import os
import tkinter as tk
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI

INTERPOLATE_NUM: int = 500  # Segments for path interpolation
DEFAULT_PLANNING_TIME: float = 5.0  # Max planning time (seconds)


def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase,
                       link: str) -> any:
    """
    Couples the gripper to a specific robot link.

    Args:
        gripper: The gripper instance.
        robot: The robot instance.
        link: The target link name.
    """
    return gripper.couple(robot, link)


def check_endeffector_upright(robot: pi.RobotBase) -> bool:
    """
    Checks if the robot's end-effector is upright within tolerance.

    Uses Euler angles to verify that the end-effector's orientation
    is near the target upright pose.

    Args:
        robot: The robot instance.

    """
    orientation = p.getEulerFromQuaternion(
        robot.get_endeffector_pose()[1]
    )
    target = np.array([-np.pi / 2, 0, 0])
    tol = np.array([0.3, 0.3, 2 * np.pi])
    return np.all(np.abs(orientation - target) <= tol)


def setup_environment() -> tuple:
    """
    Sets up the simulation environment, including URDF paths and
    PyBullet configuration.

    Returns:
        A tuple containing the robot URDF path, start position,
        start orientation, gripper URDF path, and small cube URDF path.
    """
    working_dir: str = os.path.dirname(__file__)
    urdf_robot: str = os.path.join(working_dir, 'robot_descriptions',
                                   'comau_nj290_robot.urdf')
    urdf_fofa: str = os.path.join(working_dir, 'Objects', 'FoFa', 'FoFa.urdf')
    urdf_gripper: str = os.path.join(
        working_dir, 'robot_descriptions', 'gripper_cad.urdf'
    )
    urdf_small_cube: str = os.path.join(
        working_dir, 'robot_descriptions', 'cube_small.urdf'
    )

    # Define the Comau start position and orientation.
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])

    # Connect to the PyBullet GUI.
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=50.0,
                                 cameraPitch=-30,
                                 cameraTargetPosition=(
                                     np.array([1.9, 0, 1]) + start_pos))
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000,
                                enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Load a fixed object (FoFa) into the scene.
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    return (urdf_robot, start_pos, start_orientation, urdf_gripper,
            urdf_small_cube)


def add_box(box_pos: list, half_box_size: list) -> int:
    """
    Adds a box-shaped obstacle to the simulation.

    Args:
        box_pos: The position of the box.
        half_box_size: Half the dimensions of the box.

    Returns:
        The unique ID of the created box obstacle.
    """
    col_box_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=half_box_size
    )
    box_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=box_pos,
        baseOrientation=p.getQuaternionFromEuler([-np.pi / 2, 0, 0])
    )
    return box_id


if __name__ == "__main__":
    # Set up the simulation environment.
    (urdf_robot, start_pos, start_orientation, urdf_gripper,
     urdf_cube_small) = setup_environment()

    # Create the robot instance.
    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
    start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])

    object_mover = pi.PbiObjectMover()
    gripper_mover = pi.PbiObjectMover()
    test_gripper = pi.Gripper(urdf_gripper,
                              [2.7, -0.5, 1.2],
                              start_orientation)
    position_offset = np.array([0, 0, 0])
    orientation_offset = p.getQuaternionFromEuler(np.array([-np.pi / 2, 0, 0]))
    orientation_offset = np.array(orientation_offset)

    object_mover.add_object(test_gripper.urdf, position_offset,
                            orientation_offset)
    gripper_mover.add_object(test_gripper.urdf, position_offset,
                             orientation_offset)
    # Load a small cube and set the moving object offset.
    cube_small = p.loadURDF(urdf_cube_small,
                            start_pos + [0, -2, 0],
                            useFixedBase=False)
    position_offset = np.array([0, 0.5, 0])
    object_mover.add_object(cube_small, position_offset)

    # Add a box obstacle near the robot.
    obstacles = []
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)

    # Define clearance distance for the obstacle.
    clearance_obstacles = {obstacle: 1.0}

    # Set up an initial joint state for the robot.
    initial_state = {
        'q1': -0.5,
        'q2': 0,
        'q3': -(np.pi / 2),
        'q4': 0,
        'q5': np.pi / 2,
        'q6': 0
    }
    robot.reset_joint_position(initial_state)

    # Initialize the collision checker.
    collision_checker = pi.CollisionChecker()
    position, orientation = robot.get_endeffector_pose()
    object_mover.match_moving_objects(position, orientation)
    collision_checker.set_safe_state()

    def collision_check():
        return all([collision_checker.check_collision()])

    def constraint_function():
        return all([check_endeffector_upright(robot)])

    # Maximize distanc between gripping object and obstacle
    clearance_checker = pi.CollisionChecker([obstacle, cube_small])

    # Define objectives (uncomment as needed).
    def clearance_objective(si):
        return pi.PbiPathClearanceObjective(si)

    def max_min_clearance_objective(si):
        return ob.MaximizeMinClearanceObjective(si)

    def state_cost_integral_objective(si):
        return ob.StateCostIntegralObjective(si)

    def path_length_objective(si):
        return ob.PathLengthOptimizationObjective(si)

    objective_weight: float = 1.0
    objectives: list = []
    objectives.append((clearance_objective, objective_weight))
    objectives.append((path_length_objective, objective_weight))

    def multi_objective(si):
        return pi.PbiMultiOptimizationObjective(si, objectives)

    # Define planner types.
    def rrtsharp(si):
        return og.RRTsharp(si)

    def rrt(si):
        rrt_inst = og.RRT(si)
        rrt_inst.setGoalBias(0.5)
        return rrt_inst

    def bitstar(si):
        return og.BITstar(si)

    def abitstar(si):
        return og.ABITstar(si)

    def aitstar(si):
        return og.AITstar(si)

    def get_clearance():
        min_distance = 0.3
        for (bodyA, bodyB), _ in clearance_checker.external_collision_pairs:
            curr_distance = clearance_checker.get_min_body_distance(
                bodyA, bodyB, min_distance)
            if curr_distance < min_distance:
                min_distance = curr_distance
        return min_distance

    # Initialize the path planner.
    path_planner_1 = pi.PbiPlannerSimpleSetup(
        robot=robot,
        object_mover=object_mover,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
        # constraint_functions=constraint_functions,
        # objective=objectives,
    )
    path_planner_1.name = "Robot+ Gripper+ Object"

    path_planner_2 = pi.PbiPlannerSimpleSetup(
        robot=robot,
        object_mover=gripper_mover,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
        # constraint_functions=constraint_functions,
        # objective=objectives,
    )
    path_planner_2.name = "Robot+ Gripper"

    path_planner_3 = pi.PbiPlannerSimpleSetup(
        robot=robot,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
        # constraint_functions=constraint_functions,
        # objective=objectives,
    )
    path_planner_3.name = "Solely Robot"
    path_planner = [path_planner_1, path_planner_2, path_planner_3]

    # Create the GUI for motion planning.
    root = tk.Tk()
    planner_list = [bitstar, rrt, rrtsharp, abitstar, aitstar]
    objective_list = [None, clearance_objective,
                      max_min_clearance_objective,
                      state_cost_integral_objective,
                      path_length_objective, multi_objective]
    constraint_list = [None, constraint_function]
    gui = PathPlannerGUI(root, path_planner, obstacle, planner_list, objective_list, constraint_list)

    root.mainloop()
