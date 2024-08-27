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

    working_dir = os.path.dirname(__file__)

    # Setting up the paths
    robot_view_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger', 'g_code_logger_robot_view.txt')
    joint_poisitions_path = os.path.join(
        working_dir, 'g_codes', 'g_code_logger', 'g_code_logger_joint_positions.txt')
    input_simplifier_bugs_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplifier', 'input_g_code_simplifier_bugs.txt')
    output_simplifier_bugs_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplifier', 'output_g_code_simplifier_bugs.txt')
    joint_positions_simplified_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplifier',  'joint_positions_simplified.txt')
    cartesian_simplified_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplifier', 'robot_view_simplified.txt')

    # Opening the g-codes
    with open(joint_poisitions_path, encoding='utf-8') as f:
        joint_position_g_code = f.read()
    with open(robot_view_path, encoding='utf-8') as f:
        cartesian_g_code = f.read()
    with open(input_simplifier_bugs_path, encoding='utf-8') as f:
        simplfier_bugs_g_code = f.read()

    # Reading in the g-codes
    input_g_code_joint_position = pi.GCodeProcessor.read_g_code(
        joint_position_g_code)
    input_g_code_cartesian = pi.GCodeProcessor.read_g_code(
        cartesian_g_code)
    input_g_code_bugs = pi.GCodeProcessor.read_g_code(
        simplfier_bugs_g_code)

    # Setting up the GCodeSimpflifier
    g_code_simplifier = GCodeSimplifier()

    # Demonstrating joint positions g-code simplification
    g_code_simplifier.set_g_code_and_type(
        input_g_code_joint_position, 'joint_positions')

    g_code_simplifier.simplify_g_code(0.2)
    g_code_simplifier.plot_joint_positions()

    g_code_simplifier.round_joint_positions(
        g_code_simplifier.g_code, round_dec=4)

    pi.GCodeLogger.write_g_code(
        g_code_simplifier.g_code, joint_positions_simplified_path)

    # # Demonstrating cartesion g-code simpflification
    # g_code_simplifier.set_g_code_and_type(
    #     input_g_code_cartesian, 'cartesian')

    # g_code_simplifier.simplify_g_code(0.01)
    # g_code_simplifier.plot_points()
    # g_code_simplifier.plot_orientations()
    # g_code_simplifier.round_cartesian(g_code_simplifier.g_code, 1, 3)

    # pi.GCodeLogger.write_g_code(
    #     g_code_simplifier.g_code, cartesian_simplified_path)

    # # Demonstrating bugs of the g-code simpflifier
    # g_code_simplifier.set_g_code_and_type(input_g_code_bugs, 'cartesian')
    # g_code_simplifier.simplify_g_code(0.01)
    # pi.GCodeLogger.write_g_code(
    #     g_code_simplifier.g_code, output_simplifier_bugs_path)
