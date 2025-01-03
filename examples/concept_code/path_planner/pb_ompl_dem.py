import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from robot_base import RobotBase
from pb_ompl import PbOMPL
from g_code_processor import GCodeProcessor
import utils
from itertools import product
import time


def seting_up_enviroment():
    """
    Sets up the simulation environment, including paths to URDF files and the PyBullet physics simulation.

    Returns:
        urdf_robot: Path to the robot URDF file.
        start_pos: Initial position of the robot in the simulation.
        start_orientation: Initial orientation of the robot in quaternion format.
    """
    # Define paths to robot and environment objects
    working_dir = os.path.dirname(__file__)
    urdf_robot = os.path.join(
        working_dir, '..', '..', 'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_fofa = os.path.join(
        working_dir, '..', '..', 'Objects', 'FoFa', 'FoFa.urdf')

    # Initialize the robot's starting orientation and position
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])

    # Set up PyBullet simulation
    p.connect(p.GUI, options='--background_color_red=1 ' +
              '--background_color_green=1 ' +
              '--background_color_blue=1')
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0, cameraYaw=50.0,
        cameraPitch=-30,
        cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000, enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Load environment objects
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    return urdf_robot, start_pos, start_orientation


def clear_obstacles(obstacles):
    """
    Removes all obstacles from the simulation.

    Args:
        obstacles: List of obstacle IDs to remove.
    """
    for obstacle in obstacles:
        p.removeBody(obstacle)


def add_box(box_pos, half_box_size):
    """
    Adds a box-shaped obstacle to the simulation.

    Args:
        box_pos: Position of the box center in the simulation.
        half_box_size: Half-extents of the box along each axis.

    Returns:
        box_id: ID of the created box in the simulation.
    """
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId,
                               basePosition=box_pos, baseOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
    return box_id


if __name__ == "__main__":
    # Initialize the simulation environment
    urdf_robot, start_pos, start_orientation = seting_up_enviroment()

    # Create a robot instance and position it in the simulation
    robot = RobotBase(urdf_robot, start_pos, start_orientation)
    lower_limit, upper_limit = robot.get_joint_limits()

    # Set up motion planning interface with obstacles
    obstacles = []
    pb_ompl_interface = PbOMPL(robot, obstacles)

    # Add a box obstacle to the environment
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)
    pb_ompl_interface.set_obstacles(obstacles)

    # Select a motion planner
    pb_ompl_interface.set_planner("BITstar")

    # Define start and goal configurations for the robot
    start = [-0.5, 0, -(np.pi/2), -(np.pi-0.001), -(np.pi/2), 0]
    goal = [0.5, 0.3, -(np.pi/2), -(np.pi-0.001), -(np.pi/2), 0]
    robot.set_robot(start)

    # Configure collision detection
    self_collisions = True
    pb_ompl_interface.setup_collision_detection(robot, obstacles, self_collisions)

    # Allow specific collisions and reconfigure detection
    allowed_collisions = pb_ompl_interface.get_collision_links(robot)
    pb_ompl_interface.setup_collision_detection(robot, obstacles, self_collisions, allowed_collisions)

    # Plan a path to the goal
    res, path = pb_ompl_interface.plan(goal)
    if res:
        print("Solution found. Executing path...")
        for line in path:
            robot.set_robot(line)
            time.sleep(0.01)
    else:
        print("No solution found.")
