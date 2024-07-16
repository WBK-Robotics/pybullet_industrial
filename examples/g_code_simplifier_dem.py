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

    g_code_simplifier.simplify_g_code(0.1)

    g_code_simplifier.plot_points()
    g_code_simplifier.plot_orientations()

    g_code_simplifier.round_cartesian(1, 3)

    simpflified_g_code_path = os.path.join(
        working_dir, 'g_codes', 'g_code_simplified.txt')

    pi.GCodeLogger.write_g_code(
        g_code_simplifier.g_code, simpflified_g_code_path)
