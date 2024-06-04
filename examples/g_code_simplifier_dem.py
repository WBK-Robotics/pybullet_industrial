import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from g_code_simplifier import GCodeSimplifier


def run_g_command(g_code_processor_iterator: pi.GCodeProcessor,
                  g_code_logger: pi.GCodeLogger = None):
    """
    Run the G-code commands from the given GCodeProcessor iterator.

    Args:
        g_code_processor_iterator (GCodeProcessor): Iterator yielding G-code commands.
        g_code_logger (GCodeLogger, optional): GCodeLogger instance for logging G-code changes. Defaults to None.
    """

    for _ in g_code_processor_iterator:
        # Execute the simulation steps
        for _ in range(200):
            p.stepSimulation()
        # Update G-code logger if provided
        if g_code_logger is not None:
            g_code_logger.update_g_code()


if __name__ == "__main__":

    # # Setting up the paths
    working_dir = os.path.dirname(__file__)
    # start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # urdf_robot = os.path.join(
    #     working_dir, 'robot_descriptions', 'comau_nj290_robot.urdf')

    # urdf_fofa = os.path.join(
    #     working_dir, 'Objects', 'FoFa', 'FoFa.urdf')

    # # Setting up the simulation
    # start_pos = np.array([2.0, -6.5, 0])
    # p.connect(p.GUI, options='--background_color_red=1 ' +
    #           '--background_color_green=1 ' +
    #           '--background_color_blue=1')
    # p.resetDebugVisualizerCamera(
    #     cameraDistance=2.0, cameraYaw=50.0,
    #     cameraPitch=-30,
    #     cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.setPhysicsEngineParameter(numSolverIterations=5000)
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.setGravity(0, 0, -10)
    # p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    # # Setting up robot position
    # robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
    # robot.set_joint_position(
    #     {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
    # for _ in range(100):
    #     p.stepSimulation()
    # new_point = robot.get_endeffector_pose()[0]
    # robot.set_endeffector_pose(
    #     new_point, p.getQuaternionFromEuler([0, 0, 0]))
    # for _ in range(100):
    #     p.stepSimulation()

    # # Setting up GCodeProcessor
    # g_code_processor = pi.GCodeProcessor(None, robot)

    # # Setting up GCodeLogger
    # g_code_logger = pi.GCodeLogger(robot)

    # G-Code Source
    robot_view_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger_robot_view.txt')

    joint_poisitions_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger_joint_positions.txt')

    # Setting up G-Code Simplifier
    with open(robot_view_path, encoding='utf-8') as f:
        gcode_input = f.read()
    input_g_code = pi.GCodeProcessor.read_g_code(gcode_input)

    g_code_simplifier = GCodeSimplifier(input_g_code)
    g_code_simplifier.simplify_cartesian(0.1, 0.5)
    g_code_simplifier.round_g_code(round_dec=2)

    simpflified_g_code_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplified.txt')

    pi.GCodeLogger.write_g_code(
        g_code_simplifier.g_code, simpflified_g_code_path)
