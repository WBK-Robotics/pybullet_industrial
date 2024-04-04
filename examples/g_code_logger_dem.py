import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from examples.g_code_logger import GCodeLogger


def run_g_command(g_code_processor_iterator: pi.GCodeProcessor,
                  g_code_logger: GCodeLogger = None):
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
            g_code_logger.update()


if __name__ == "__main__":

    # # Setting up the paths
    # working_dir = os.path.dirname(__file__)
    # pb_base_path = "C:\\MaH\\pybullet_industrial\\examples"
    # pb_base_path = "C:/Users/phili/OneDrive/Dokumente/GitHub/pybullet_industrial/examples"

    # start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # urdf_robot = os.path.join(
    #     working_dir, 'robot_descriptions', 'comau_NJ290_robotNC.urdf')

    # urdf_fofa = os.path.join(
    #     pb_base_path, 'Objects', 'FoFa', 'FoFa.urdf')

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

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')

    urdf_file3 = os.path.join(dirname,
                              'Objects', 'FoFa', 'FoFa.urdf')

    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                             '--background_color_green=1 ' +
                                             '--background_color_blue=1')
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    p.loadURDF(urdf_file3, [-2, 5, 0], useFixedBase=True, globalScaling=0.001)

    test_robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])

    test_robot.set_joint_position(({'q2': np.deg2rad(-15.0),
                                  'q3': np.deg2rad(-90.0)}))
    for _ in range(100):
        p.stepSimulation()

    # Setting up GCodeProcessor
    g_code_processor = pi.GCodeProcessor(None, test_robot)

    # Setting up GCodeLogger
    g_code_logger = GCodeLogger(test_robot)
    g_code_test = os.path.join(
        working_dir, 'g_codes', 'g_code_logger_input.txt')
    with open(g_code_test, encoding='utf-8') as f:
        gcode_input = f.read()
    g_code_processor.g_code = g_code_processor.read_g_code(gcode_input)

    # Setting first g-code line
    full_g_code = g_code_processor.g_code
    first_entry = [g_code_processor.g_code[0]]
    iterator = iter(g_code_processor)

    # Run first enty
    g_code_processor.g_code = first_entry
    run_g_command(iterator)

    # Run demonstration sequence and record g-codes
    g_code_processor.g_code = full_g_code
    run_g_command(iterator, g_code_logger)

    # Run g_code_joint_position
    g_code_processor.g_code = g_code_logger.g_code_joint_position
    run_g_command(g_code_processor)

    # Run g_code_robot_view
    g_code_processor.g_code = g_code_logger.g_code_robot_view
    run_g_command(g_code_processor)

    # Writing g_code
    robot_view_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger_robot_view.txt')

    joint_poisitions_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger_joint_positions.txt')

    g_code_logger.write_g_code(
        g_code_logger.g_code_robot_view, robot_view_path)
    g_code_logger.write_g_code(
        g_code_logger.g_code_joint_position, joint_poisitions_path)
