import copy
import os
import tkinter as tk
import numpy as np
import pybullet as p
import pybullet_data
from ompl import base as ob
from ompl import geometric as og
from scipy.spatial.transform import Rotation as R
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI


def transform_eulers_in_gcode(g_code: list) -> list:
    """
    Converts Euler angles in a G-code list from PyBullet to Siemens.

    A deep copy of the input is performed. For each command that contains
    keys 'A', 'B', and 'C', the corresponding Euler angles are converted
    from 'xyz' to 'XYZ' convention.

    Args:
        g_code (list): List of G-code commands.

    Returns:
        list: Transformed G-code commands.
    """
    return_code = copy.deepcopy(g_code)
    for command in return_code:
        if 'A' in command and 'B' in command and 'C' in command:
            euler_pb = np.array([command['A'],
                                 command['B'],
                                 command['C']])
            rot = R.from_euler('xyz', euler_pb)
            euler_siemens = rot.as_euler('XYZ')
            command['A'] = euler_siemens[0]
            command['B'] = euler_siemens[1]
            command['C'] = euler_siemens[2]
    return return_code


def check_endeffector_upright(robot: pi.RobotBase) -> bool:
    """
    Checks if the robot's end-effector is upright within a tolerance.

    Euler angles are used to verify that the orientation is near
    the upright pose.

    Args:
        robot (pi.RobotBase): The robot instance.

    Returns:
        bool: True if the end-effector is upright, False otherwise.
    """
    quat = robot.get_endeffector_pose()[1]
    orientation = p.getEulerFromQuaternion(quat)
    target = np.array([0, 0, 0])
    tol = np.array([0.05, 0.05, 2 * np.pi])
    return np.all(np.abs(orientation - target) <= tol)


def setup_envirnoment(working_dir: str):
    """
    Configures the simulation environment.

    URDF paths are defined and objects, robots, and other elements
    are loaded into the PyBullet simulation.

    Args:
        working_dir (str): Base directory for resources.

    Returns:
        tuple: (robots, gripper, objects)
    """
    # Define URDF file paths.
    urdf_fofa = os.path.join(
        working_dir, 'Objects', 'FoFa', 'FoFa.urdf'
    )
    urdf_comau = os.path.join(
        working_dir, 'robot_descriptions',
        'comau_nj290_robotNC.urdf'
    )
    urdf_table = os.path.join(
        working_dir, 'Objects', "Spannplatte.urdf"
    )
    urdf_SRG = os.path.join(
        working_dir, 'robot_descriptions', 'SRG.urdf'
    )
    urdf_emo_sr = os.path.join(
        working_dir, 'Objects', 'EMO_SR_full.urdf'
    )
    urdf_box = os.path.join(
        working_dir, 'Objects', 'Box', 'stor_box.urdf'
    )
    urdf_fixture = os.path.join(
        working_dir, 'Objects', 'Dreibackenfutter',
        'Dreibackenfutter.urdf'
    )

    # Configure camera and visual parameters.
    p.resetDebugVisualizerCamera(
        cameraDistance=2.75,
        cameraYaw=35.0,
        cameraPitch=-30,
        cameraTargetPosition=np.array([0, 0, 0.5])
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=2000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load background object and set gravity.
    p.loadURDF(
        urdf_fofa,
        np.array([-3.1310 - 1, 6.5, 0]),
        useFixedBase=True,
        globalScaling=0.001
    )
    p.setGravity(0, 0, -10)

    # -------------------------------
    # Robots and Objects Loading
    # -------------------------------
    # Robot C.
    start_pos_robot_C = np.array([2.135, 0, 0])
    start_ori_robot_C = p.getQuaternionFromEuler([0, 0, np.pi])
    robot_C = pi.RobotBase(
        urdf_comau, start_pos_robot_C, start_ori_robot_C
    )

    # Robot D.
    start_pos_robot_D = np.array([-2.135, 0, 0])
    start_ori_robot_D = p.getQuaternionFromEuler([0, 0, 0])
    robot_D = pi.RobotBase(
        urdf_comau, start_pos_robot_D, start_ori_robot_D
    )

    # Gripper.
    start_pos_grip = np.array([0.9, -0.9, 0.5])
    start_ori_grip = p.getQuaternionFromEuler([0, 0, 0])
    srg_gripper = pi.Gripper(
        urdf_SRG, start_pos_grip, start_ori_grip
    )

    # Load objects.
    pos_box = np.array([0.4, 0.3, 0.261])
    box = p.loadURDF(
        urdf_box, pos_box, useFixedBase=True, globalScaling=2.5
    )
    emo_sr = p.loadURDF(
        urdf_emo_sr, pos_box + [0.2, -0.4, 0],
        useFixedBase=True, globalScaling=0.001
    )

    spawn_point_fixture = [
        -0.00954569224268198, 0.014756819233298302, 0.2662343382835388
    ]
    spawn_ori_fixture = p.getQuaternionFromEuler(
        np.array([-0.00756672225243951, 0.011473507510403333,
                  1.3312571405821847])
    )
    fixture = p.loadURDF(
        urdf_fixture, spawn_point_fixture, spawn_ori_fixture,
        useFixedBase=True, globalScaling=0.001
    )
    table = p.loadURDF(
        urdf_table,
        np.array([-1, -0.685, 0.0]),
        useFixedBase=True, globalScaling=0.001
    )

    # Create a wall.
    wall_pos = [0.4, 0, 0.3]
    wall_size = [0.5, 0.5, 0.05]
    col_box_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=wall_size
    )
    wall = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=wall_pos,
        baseOrientation=p.getQuaternionFromEuler([-np.pi / 2, 0, 0])
    )
    p.changeVisualShape(
        wall, -1, rgbaColor=[0.8, 0.9, 1, 0.8]
    )

    # -------------------------------
    # Path Planner Setup
    # -------------------------------
    # Define initial joint states.
    initial_state_C = {
        'q1': 0.16,
        'q2': 0.14,
        'q3': -1.69,
        'q4': 0,
        'q5': 1.3,
        'q6': -0.3
    }
    initial_state_D = {
        'q1': -0.5,
        'q2': 0,
        'q3': -(np.pi / 2),
        'q4': 0,
        'q5': np.pi / 2,
        'q6': 0
    }
    robot_C.reset_joint_position(initial_state_C)
    robot_D.reset_joint_position(initial_state_D)

    gripper = [srg_gripper]
    robots = [robot_C, robot_D]
    objects = [emo_sr, box, wall, table, fixture, srg_gripper.urdf]

    return robots, gripper, objects


def setup_planner_gui(robots, gripper, objects):
    """
    Configures and launches the path planner GUI.

    Object movers are created and objects are added with appropriate
    offsets. Collision checkers, objectives, and planner types are defined.
    The GUI is then started.

    Args:
        robots (list): List of robot instances.
        gripper (list): List of gripper instances.
        objects (list): List of objects in the simulation.

    Returns:
        tuple: (joint_path, g_code_logger)
    """
    # Create object movers.
    object_mover = pi.PbiObjectMover()
    gripper_mover = pi.PbiObjectMover()

    # Add gripper to object movers.
    pos_offset = np.array([0, 0, 0])
    ori_offset = np.array(p.getQuaternionFromEuler([0, 0, 0]))
    object_mover.add_object(gripper[0].urdf, pos_offset, ori_offset)
    gripper_mover.add_object(gripper[0].urdf, pos_offset, ori_offset)

    # Add EMO_SR with an offset.
    pos_offset = np.array([0, 0, -0.35])
    object_mover.add_object(objects[0], pos_offset)

    # Configure collision checkers.

    # Settiing up collision checkers for the robot C
    collision_checker_C = pi.CollisionChecker()
    collision_checker_C.make_robot_static(robots[1].urdf)

    # Setting up collision checkers for the robot D
    collision_checker_D = pi.CollisionChecker()
    collision_checker_D.make_robot_static(robots[0].urdf)

    # Clearance object motor and wall
    motor_clearance = pi.CollisionChecker([objects[0],
                                           objects[2]])
    # Clearance object for both robots
    robot_clearance = pi.CollisionChecker([robots[0].urdf,
                                           robots[1].urdf])
    robot_clearance.remove_link_id(robots[0].urdf, -1)
    robot_clearance.remove_link_id(robots[0].urdf, 0)
    robot_clearance.remove_link_id(robots[0].urdf, 1)
    robot_clearance.remove_link_id(robots[0].urdf, 2)
    robot_clearance.remove_link_id(robots[0].urdf, 3)
    robot_clearance.remove_link_id(robots[0].urdf, 4)

    robot_clearance.make_robot_static(robots[0].urdf)
    robot_clearance.make_robot_static(robots[1].urdf)

    # Setting safe state for robot C + gripper + object
    pos, ori = robots[0].get_endeffector_pose()
    object_mover.match_moving_objects(pos, ori)
    collision_checker_C.set_safe_state()
    collision_checker_D.set_safe_state()

    def collision_check_C() -> bool:
        """Returns True if collisions are absent."""
        return all([collision_checker_C.is_collision_free()])

    def collision_check_D() -> bool:
        """Returns True if collisions are absent."""
        return all([collision_checker_D.is_collision_free()])

    def constraint_function() -> bool:
        """Returns True if the end-effector is upright."""
        return all([check_endeffector_upright(robots[0])])

    def constraint_function_robot_D() -> bool:
        """Returns True if the end-effector is upright."""
        return all([check_endeffector_upright(robots[1])])

    # Define objective functions.
    def maximize_min_clearance_objective(si):
        return pi.PbiMaximizeMinClearanceObjective(si)

    def clearance_objective(si):
        return pi.PbiClearanceObjective(si)

    def joint_path_length_objective(si):
        return ob.PathLengthOptimizationObjective(si)

    def endeffector_path_length_objective(si):
        return pi.PbiEndeffectorPathLengthObjective(si)

    objective_weight: float = 0.5
    objectives = []
    objectives.append((endeffector_path_length_objective, objective_weight))
    objectives.append((clearance_objective, 1 - objective_weight))

    def endeffector_path_clearance_objective(si):
        return pi.PbiMultiOptimizationObjective(si, objectives)

    objective_weight: float = 0.5
    objectives = []
    objectives.append((joint_path_length_objective, objective_weight))
    objectives.append((clearance_objective, 1 - objective_weight))

    def joint_path_clearance_objective(si):
        return pi.PbiMultiOptimizationObjective(si, objectives)

    # Define planner types.
    def rrtsharp(si):
        return og.RRTsharp(si)

    def rrt(si):
        rrt_inst = og.RRT(si)
        rrt_inst.setGoalBias(0.2)
        return rrt_inst

    def bitstar(si):
        return og.BITstar(si)

    def abitstar(si):
        return og.ABITstar(si)

    def aitstar(si):
        return og.AITstar(si)

    def informed_rrtstar(si):
        return og.InformedRRTstar(si)

    def rrtstar(si):
        return og.RRTstar(si)

    def aitstar(si):
        return og.AITstar(si)

    def rrtconnect(si):
        return og.RRTConnect(si)

    def sbl(si):
        return og.SBL(si)

    def fmt(si):
        return og.FMT(si)

    def bfmt(si):
        return og.BFMT(si)

    def lbkpiece1(si):
        return og.LBKPIECE1(si)

    def get_motor_clearance():
        return motor_clearance.get_external_distance(3)

    def get_robot_clearance():
        return robot_clearance.get_external_distance(0.5)

    # Initialize planner setups.
    path_planner_1 = pi.PbiPlannerSimpleSetup(
        robot=robots[0],
        object_mover=object_mover,
        collision_check_function=collision_check_C,
        clearance_function=get_motor_clearance
    )
    path_planner_1.name = "Robot+ Gripper+ Object"

    path_planner_2 = pi.PbiPlannerSimpleSetup(
        robot=robots[0],
        object_mover=gripper_mover,
        collision_check_function=collision_check_C,
        # clearance_function=get_robot_clearance
    )
    path_planner_2.name = "Robot+ Gripper"

    path_planner_3 = pi.PbiPlannerSimpleSetup(
        robot=robots[0],
        collision_check_function=collision_check_C,
        clearance_function=get_robot_clearance
    )
    path_planner_3.name = "Solely Robot"

    path_planner_4 = pi.PbiPlannerSimpleSetup(
        robot=robots[1],
        collision_check_function=collision_check_D,
        clearance_function=get_robot_clearance
    )
    path_planner_4.name = "2nd Solely Robot"

    # -------------------------------
    # GUI Setup
    # -------------------------------
    path_planner_list = [
        path_planner_1, path_planner_2, path_planner_3,
        path_planner_4, path_planner_4
    ]
    planner_list = [
        abitstar, bitstar, rrt, rrtstar, informed_rrtstar]
    objective_list = [
        None, clearance_objective, endeffector_path_length_objective,
        joint_path_length_objective, endeffector_path_clearance_objective,
        joint_path_clearance_objective
    ]
    constraint_list = [None, constraint_function, constraint_function_robot_D]

    # Create and run the GUI.
    root = tk.Tk()
    gui = PathPlannerGUI(root, path_planner_list, objects,
                         planner_list, objective_list,
                         constraint_list)
    root.mainloop()
    joint_path = copy.deepcopy(gui.joint_path)
    g_code_logger: pi.GCodeLogger = gui.g_code_logger
    return joint_path, g_code_logger


def transform_g_code(g_code):
    """
    Processes G-code with a series of transformations.

    The G-code is processed to convert Euler angles, scale coordinates,
    convert angles to degrees, round pose values, and apply feedrate.

    Args:
        g_code: Raw G-code instructions.

    Returns:
        Processed G-code.
    """
    g_code = transform_eulers_in_gcode(g_code)
    g_code = pi.GCodeSimplifier.scale_g_code(
        g_code, 1000.0, ['X', 'Y', 'Z']
    )
    g_code = pi.GCodeSimplifier.convert_to_degrees(g_code)
    g_code = pi.GCodeSimplifier.round_pose_values(g_code, 4, 4)
    g_code = pi.GCodeSimplifier.apply_feedrate(g_code, 5000)
    return g_code


if __name__ == "__main__":
    p.connect(p.GUI)
    working_dir: str = os.path.dirname(__file__)

    robots, gripper, objects = setup_envirnoment(working_dir)

    _, g_code_logger = setup_planner_gui(robots, gripper, objects)

    se3_g_code = g_code_logger.g_code_robot_view
    transformed_g_code = transform_g_code(se3_g_code)

    exportfile = os.path.join(
        working_dir, 'g_codes', 'transformed.mpf'
    )
    pi.GCodeLogger.write_g_code(
        transformed_g_code, exportfile, {}, postfix="M30\n"
    )
