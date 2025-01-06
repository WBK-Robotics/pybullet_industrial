import os
import pybullet as p
import pybullet_data
import numpy as np
from robot_base import RobotBase
from path_planner import PathPlanner
from collision_checker import CollisionChecker
import time


def seting_up_enviroment():
    """
    Sets up the simulation environment, including paths to URDF files
    and the PyBullet physics simulation.

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

    # Add a box obstacle to the environment
    obstacles = []
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)

    # Initialize CollisionChecker
    collision_checker = CollisionChecker(robot, obstacles)

    # Set up motion planning interface with the CollisionChecker
    path_planner = PathPlanner(robot, collision_checker, "BITstar")

    # Define start and goal configurations for the robot
    start = {'q1': -0.5, 'q2': 0, 'q3': -(np.pi/2),
             'q4': -(np.pi-0.001), 'q5': -(np.pi/2), 'q6': 0}
    goal = {'q1': 0.5, 'q2': 0.3, 'q3': -(np.pi/2),
            'q4': -(np.pi-0.001), 'q5': -(np.pi/2), 'q6': 0}
    robot.reset_joint_position(start)

    # Allow specific collisions and reconfigure detection
    collision_checker.allow_collision_links = collision_checker.get_collision_links()
    collision_checker.update_collision_settings()

    # Plan a path to the goal
    res, joint_path = path_planner.plan_start_goal(start, goal)
    if res:
        for target, _ in joint_path:
            robot.reset_joint_position(target)
            time.sleep(0.01)
    else:
        print("No solution found.")
