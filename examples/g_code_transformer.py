import copy
import tkinter as tk
import numpy as np
import os
import pybullet_data
import matplotlib.pyplot as plt
from ompl import base as ob
from ompl import geometric as og
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI


def transform_eulers_in_gcode(g_code: list):
    return_code = copy.deepcopy(g_code)
    for command in return_code:
        if 'A' in command and 'B' in command and 'C' in command:
            euler_pb = np.array([command['A'], command['B'], command['C']])
            rot = R.from_euler('xyz', euler_pb)
            euler_siemens = rot.as_euler('XYZ')
            command['A'] = euler_siemens[0]
            command['B'] = euler_siemens[1]
            command['C'] = euler_siemens[2]
    return return_code


if __name__ == "__main__":
    # Pybulllet Environment
    working_dir: str = os.path.dirname(__file__)
    urdf_fofa: str = os.path.join(working_dir,
                                  'Objects', 'FoFa', 'FoFa.urdf')
    urdf_comau: str = os.path.join(working_dir, 'robot_descriptions',
                                   'comau_nj290_robotNC.urdf')
    urdf_store_box = os.path.join(working_dir,
                                  'Objects', 'Box', 'stor_box.urdf')
    urdf_table = os.path.join(working_dir, 'Objects', "Spannplatte.urdf")
    urdf_SRG: str = os.path.join(
        working_dir, 'robot_descriptions', 'SRG.urdf')
    urdf_cube_small: str = os.path.join(
        working_dir, 'robot_descriptions', 'cube_small.urdf')
    urdf_box = os.path.join(working_dir, 'Objects', 'Box', 'stor_box.urdf')
    # urdf_zivid = os.path.join(working_dir,
    #                           'robot_descriptions', 'zivid.urdf')
    urdf_fixture = os.path.join(working_dir, 'Objects', 'Dreibackenfutter',
                                'Dreibackenfutter.urdf')

    # opengl2 is necessary to sidestep problem of maximal rendered triangles
    pysics_client = p.connect(p.GUI)

    p.resetDebugVisualizerCamera(cameraDistance=2.75, cameraYaw=35.0,
                                 cameraPitch=-30,
                                 cameraTargetPosition=np.array([0, 0, 0.5]))
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF(urdf_fofa, np.array(
        [-3.1310-1, 6.5, 0]), useFixedBase=True, globalScaling=0.001)

    p.setGravity(0, 0, -10)

    # Robots and endeffectors
    start_pos_robot_C = np.array([-2.135, 0, 0])
    start_orientation_robot_C = p.getQuaternionFromEuler([0, 0, 0])
    robot_C = pi.RobotBase(urdf_comau, start_pos_robot_C,
                           start_orientation_robot_C)

    start_pos_robot_D = np.array([2.135, 0, 0])
    start_orientation_robot_D = p.getQuaternionFromEuler([0, 0, np.pi])
    robot_D = pi.RobotBase(urdf_comau, start_pos_robot_D,
                           start_orientation_robot_D)

    start_pos_grip = np.array([0.9, -0.9, 0.5])
    start_orientation_grip = p.getQuaternionFromEuler([0, 0, 0])
    SRG = pi.Gripper(urdf_SRG, start_pos_grip, start_orientation_grip)

    posBox = np.array([0.4, -0.1, 0.261])
    box  = p.loadURDF(urdf_box, posBox, useFixedBase=True)
    cube_small = p.loadURDF(urdf_cube_small,
                            posBox + [0, -0.2, 0.1],
                            useFixedBase=False)

    spawn_point_fixture = [-0.00954569224268198,
                           0.014756819233298302, 0.2662343382835388]
    spawn_orient_fixture = p.getQuaternionFromEuler(
        np.array([-0.00756672225243951, 0.011473507510403333, 1.3312571405821847]))
    fixture = p.loadURDF(urdf_fixture,
                         spawn_point_fixture, spawn_orient_fixture, useFixedBase=True,
                         globalScaling=0.001)
    table = p.loadURDF(urdf_table,
               np.array([-1, -0.685, 0.0]), useFixedBase=True, globalScaling=0.001)

    # Setting up PathPlanner
    collision_checker = pi.CollisionChecker()
    # position, orientation = robot.get_endeffector_pose()
    # object_mover.match_moving_objects(position, orientation)
    collision_checker.set_safe_state()

    def collision_check():
        return all([collision_checker.check_collision()])

    def bitstar(si):
        return og.BITstar(si)

    path_planner_3 = pi.PbiPlannerSimpleSetup(
        robot=robot_C,
        collision_check_function=collision_check,
        planner_type=bitstar,
        # clearance_function=get_clearance
        # constraint_functions=constraint_functions,
        # objective=objectives,
    )
    path_planner_3.name = "Solely Robot"
    path_planner = [# path_planner_1, path_planner_2,
                    path_planner_3]
    planner_list = [bitstar]
    constraint_list = [None]
    objective_list = [None]
    # Setting up GUI
    root = tk.Tk()
    gui = PathPlannerGUI(root, path_planner, [box, cube_small], planner_list, objective_list, constraint_list)
    root.mainloop()


    processor = pi.GCodeProcessor(robot=robot_C)

    # Read in G-Code
    textfile = os.path.join(working_dir, 'g_codes',
                            'joint_path_planner.txt')

    with open(textfile, encoding='utf-8') as f:
        gcode_input = f.read()
    processor.g_code = processor.read_g_code(gcode_input)

    # Transform G-Code
    processor.g_code = transform_eulers_in_gcode(processor.g_code)
    processor.g_code = pi.GCodeSimplifier.add_offset_to_g_code(
        processor.g_code, {'X': -1, 'Y': 6.5, 'Z': 0.0})
    processor.g_code = pi.GCodeSimplifier.scale_g_code(
        processor.g_code, 1000.0, ['X', 'Y', 'Z'])
    processor.g_code = pi.GCodeSimplifier.convert_to_degrees(processor.g_code)
    processor.g_code = pi.GCodeSimplifier.round_cartesian(
        processor.g_code, 4, 4)
    processor.g_code = pi.GCodeSimplifier.apply_feedrate(
        processor.g_code, 5000)
    exportfile = os.path.join(working_dir, 'g_codes',
                              'transformed.mpf')
    pi.GCodeLogger.write_g_code(
        processor.g_code, exportfile, {},
        postfix="M30\n")
