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


# =============================================================================
# Helper Functions
# =============================================================================
def transform_eulers_in_gcode(g_code: list):
    """
    Transform Euler angles from one convention to another in a G-code list.

    This function deep copies the input G-code, then converts the 'A', 'B', 'C'
    values (assumed to be Euler angles in PyBullet convention) into Siemens
    convention (as an example) using scipy's Rotation module.
    """
    return_code = copy.deepcopy(g_code)
    for command in return_code:
        if ('A' in command and 'B' in command and
                'C' in command):
            euler_pb = np.array([command['A'], command['B'], command['C']])
            rot = R.from_euler('xyz', euler_pb)
            euler_siemens = rot.as_euler('XYZ')
            command['A'] = euler_siemens[0]
            command['B'] = euler_siemens[1]
            command['C'] = euler_siemens[2]
    return return_code


def check_endeffector_upright(robot: pi.RobotBase) -> bool:
    """
    Check if the robot's end-effector is upright within tolerance.

    Uses Euler angles to verify that the end-effector's orientation is near
    the target upright pose.

    Args:
        robot: The robot instance.
    """
    orientation = p.getEulerFromQuaternion(
        robot.get_endeffector_pose()[1]
    )
    target = np.array([-np.pi / 2, 0, 0])
    tol = np.array([0.3, 0.3, 2 * np.pi])
    return np.all(np.abs(orientation - target) <= tol)


# =============================================================================
# Main Script
# =============================================================================
if __name__ == "__main__":
    # -------------------------------
    # Environment Setup
    # -------------------------------
    working_dir: str = os.path.dirname(__file__)

    # Define URDF file paths for objects and robots.
    urdf_fofa = os.path.join(working_dir, 'Objects', 'FoFa', 'FoFa.urdf')
    urdf_comau = os.path.join(
        working_dir, 'robot_descriptions', 'comau_nj290_robotNC.urdf'
    )
    urdf_store_box = os.path.join(
        working_dir, 'Objects', 'Box', 'stor_box.urdf'
    )
    urdf_table = os.path.join(working_dir, 'Objects', "Spannplatte.urdf")
    urdf_SRG = os.path.join(
        working_dir, 'robot_descriptions', 'SRG.urdf'
    )
    urdf_cube_small = os.path.join(
        working_dir, 'robot_descriptions', 'cube_small.urdf'
    )
    urdf_box = os.path.join(
        working_dir, 'Objects', 'Box', 'stor_box.urdf'
    )
    urdf_fixture = os.path.join(
        working_dir, 'Objects', 'Dreibackenfutter',
        'Dreibackenfutter.urdf'
    )

    # Connect to PyBullet with GUI and configure visual parameters.
    physics_client = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=2.75,
        cameraYaw=35.0,
        cameraPitch=-30,
        cameraTargetPosition=np.array([0, 0, 0.5])
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load background object (FoFa) and set gravity.
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
    # --- Robot Loading ---
    start_pos_robot_C = np.array([-2.135, 0, 0])
    start_orientation_robot_C = p.getQuaternionFromEuler([0, 0, 0])
    robot_C = pi.RobotBase(
        urdf_comau, start_pos_robot_C, start_orientation_robot_C
    )

    start_pos_robot_D = np.array([2.135, 0, 0])
    start_orientation_robot_D = p.getQuaternionFromEuler([0, 0, np.pi])
    robot_D = pi.RobotBase(
        urdf_comau, start_pos_robot_D, start_orientation_robot_D
    )

    start_pos_grip = np.array([0.9, -0.9, 0.5])
    start_orientation_grip = p.getQuaternionFromEuler([0, 0, 0])
    srg_gripper = pi.Gripper(
        urdf_SRG, start_pos_grip, start_orientation_grip
    )

    # --- Object Loading ---
    posBox = np.array([0.4, -0.1, 0.261])
    box = p.loadURDF(
        urdf_box, posBox, useFixedBase=True
    )
    cube_small = p.loadURDF(
        urdf_cube_small, posBox + [0, -0.2, 0.1],
        useFixedBase=False
    )

    spawn_point_fixture = [-0.00954569224268198, 0.014756819233298302,
                           0.2662343382835388]
    spawn_orient_fixture = p.getQuaternionFromEuler(
        np.array([-0.00756672225243951, 0.011473507510403333,
                  1.3312571405821847])
    )
    fixture = p.loadURDF(
        urdf_fixture,
        spawn_point_fixture,
        spawn_orient_fixture,
        useFixedBase=True,
        globalScaling=0.001
    )
    table = p.loadURDF(
        urdf_table,
        np.array([-1, -0.685, 0.0]),
        useFixedBase=True,
        globalScaling=0.001
    )

    # -------------------------------
    # Path Planner Setup
    # -------------------------------
    # Reset initial joint states.
    initial_state = {
        'q1': -0.5,
        'q2': 0,
        'q3': -(np.pi / 2),
        'q4': 0,
        'q5': np.pi / 2,
        'q6': 0
    }
    robot_C.reset_joint_position(initial_state)
    robot_D.reset_joint_position(initial_state)

    # Create object movers for the planner.
    object_mover = pi.PbiObjectMover()
    gripper_mover = pi.PbiObjectMover()

    # Add objects to the object movers.
    position_offset = np.array([0, 0, 0])
    orientation_offset = np.array(
        p.getQuaternionFromEuler([0, 0, 0])
    )
    object_mover.add_object(
        srg_gripper.urdf, position_offset, orientation_offset
    )
    gripper_mover.add_object(
        srg_gripper.urdf, position_offset, orientation_offset
    )

    # Add cube_small with an offset.
    position_offset = np.array([0, 0, -0.2])
    object_mover.add_object(cube_small, position_offset)

    # Configure collision checking.
    collision_checker = pi.CollisionChecker()
    position, orientation = robot_C.get_endeffector_pose()
    object_mover.match_moving_objects(position, orientation)
    collision_checker.set_safe_state()

    def collision_check():
        """Return True if no collisions are detected."""
        return all([collision_checker.check_collision()])

    def constraint_function():
        """Return True if the end-effector is upright."""
        return all([check_endeffector_upright(robot_C)])

    # Define objective functions.
    def clearance_objective(si):
        return pi.PbiPathClearanceObjective(si)

    def max_min_clearance_objective(si):
        return ob.MaximizeMinClearanceObjective(si)

    def state_cost_integral_objective(si):
        return ob.StateCostIntegralObjective(si)

    def path_length_objective(si):
        return ob.PathLengthOptimizationObjective(si)

    objective_weight: float = 1.0
    objectives = []
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
        return collision_checker.get_global_distance(0.3)

    # Initialize multiple planner setups.
    path_planner_1 = pi.PbiPlannerSimpleSetup(
        robot=robot_C,
        object_mover=object_mover,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
    )
    path_planner_1.name = "Robot+ Gripper+ Object"

    path_planner_2 = pi.PbiPlannerSimpleSetup(
        robot=robot_C,
        object_mover=gripper_mover,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
    )
    path_planner_2.name = "Robot+ Gripper"

    path_planner_3 = pi.PbiPlannerSimpleSetup(
        robot=robot_C,
        collision_check_function=collision_check,
        planner_type=bitstar,
        clearance_function=get_clearance
    )
    path_planner_3.name = "Solely Robot"

    # -------------------------------
    # GUI Setup
    # -------------------------------
    # Prepare lists for planner setups, planner types, objectives, and
    # constraints.
    path_planner_list = [path_planner_1, path_planner_2, path_planner_3]
    planner_list = [bitstar, rrt, rrtsharp, abitstar, aitstar]
    objective_list = [
        None, clearance_objective, state_cost_integral_objective,
        path_length_objective, multi_objective
    ]
    constraint_list = [None, constraint_function]

    # Create and run the GUI.
    root = tk.Tk()
    gui = PathPlannerGUI(
        root, path_planner_list, [box, cube_small],
        planner_list, objective_list, constraint_list
    )
    root.mainloop()

    # -------------------------------
    # G-Code Processing and Export
    # -------------------------------
    # Create a G-code processor for robot_C.
    processor = pi.GCodeProcessor(robot=robot_C)

    # Read in G-code from file.
    textfile = os.path.join(
        working_dir, 'g_codes', 'joint_path_planner.txt'
    )
    with open(textfile, encoding='utf-8') as f:
        gcode_input = f.read()
    processor.g_code = processor.read_g_code(gcode_input)

    # Process the G-code with several transformations.
    processor.g_code = transform_eulers_in_gcode(processor.g_code)
    processor.g_code = pi.GCodeSimplifier.add_offset_to_g_code(
        processor.g_code, {'X': -1, 'Y': 6.5, 'Z': 0.0}
    )
    processor.g_code = pi.GCodeSimplifier.scale_g_code(
        processor.g_code, 1000.0, ['X', 'Y', 'Z']
    )
    processor.g_code = pi.GCodeSimplifier.convert_to_degrees(
        processor.g_code
    )
    processor.g_code = pi.GCodeSimplifier.round_cartesian(
        processor.g_code, 4, 4
    )
    processor.g_code = pi.GCodeSimplifier.apply_feedrate(
        processor.g_code, 5000
    )

    # Export the transformed G-code to a file.
    exportfile = os.path.join(
        working_dir, 'g_codes', 'transformed.mpf'
    )
    pi.GCodeLogger.write_g_code(
        processor.g_code, exportfile, {}, postfix="M30\n"
    )
