import os
import tkinter as tk
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI

INTERPOLATE_NUM = 500  # Number of segments for interpolating the path
DEFAULT_PLANNING_TIME = 5.0  # Maximum planning time in seconds


def check_endeffector_upright(robot: pi.RobotBase):
    """Checks if the robot's end-effector is upright within tolerance.

    Returns:
        bool: True if upright, False otherwise.
    """
    orientation = p.getEulerFromQuaternion(
        robot.get_endeffector_pose()[1]
    )
    target = np.array([-np.pi / 2, 0, 0])
    tol = np.array([0.3, 0.3, 2 * np.pi])
    return np.all(np.abs(orientation - target) <= tol)

def stateCost(collision_checker_list):
    """Computes the cost of a state based on its clearance.

    Args:
        s (ob.State): The state for which to compute the cost.

    Returns:
        ob.Cost: The computed cost.
    """

    state_cost = [cc.get_min_body_distance(bodyA=0, bodyB=1, distance=1) for cc in collision_checker_list]

    return min(state_cost)


def seting_up_enviroment():
    """
    Sets up the simulation environment, including paths to URDF files and
    the PyBullet physics simulation.
    """
    working_dir = os.path.dirname(__file__)
    urdf_robot = os.path.join(working_dir, 'robot_descriptions',
                              'comau_nj290_robot.urdf')
    urdf_fofa = os.path.join(working_dir, 'Objects', 'FoFa', 'FoFa.urdf')

    # Comau start position.
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])

    # Set up the GUI camera.
    p.connect(p.GUI,
              options='--background_color_red=1 '
                      '--background_color_green=1 '
                      '--background_color_blue=1')
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=50.0,
                                 cameraPitch=-30,
                                 cameraTargetPosition=(
                                     np.array([1.9, 0, 1]) + start_pos))

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000,
                                enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    return urdf_robot, start_pos, start_orientation


def add_box(box_pos, half_box_size):
    """
    Adds a box-shaped obstacle to the simulation.
    """
    colBoxId = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=colBoxId,
        basePosition=box_pos,
        baseOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0])
    )
    return box_id


if __name__ == "__main__":
    # Initialize the simulation environment.
    urdf_robot, start_pos, start_orientation = seting_up_enviroment()

    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)

    # Add a box obstacle.
    obstacles = []

    # Create a box obstacle near the robot.
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)

    # Define custom clearance query distances for each obstacle.
    # Here, we set a clearance query distance of 1.0 for the obstacle.
    clearance_obstacles = {obstacle: 1.0}

    # Initialize CollisionChecker with the custom clearance.
    ignored_urdfs = [0] # ignore Fofa
    collision_checker = pi.CollisionChecker(ignored_urdfs)
    collision_checker.set_safe_state()
    internal_collision = collision_checker.check_internal_collisions()
    external_collision = collision_checker.check_external_collisions()
    global_collision = collision_checker.check_collision()

    # Append constraint functinons
    constraint_functions = [lambda: check_endeffector_upright(robot)]
    state_cost = [lambda: stateCost([collision_checker])]
    # Initialize PathPlanner with the clearance objective.
    path_planner = pi.PathPlanner(robot=robot, collision_checker_list=[collision_checker], planner_name="BITstar",
                                  objective="pathclearance",
                                  constraint_functions=constraint_functions,
                                  state_cost=state_cost,)

    # Set up initial state (for Comau).
    inital_state = {
        'q1': -0.5,
        'q2': 0,
        'q3': -(np.pi/2),
        'q4': 0,
        'q5': np.pi/2,
        'q6': 0
    }
    robot.reset_joint_position(inital_state)

    # Create the GUI for motion planning.
    root = tk.Tk()
    gui = PathPlannerGUI(root, robot, path_planner, [collision_checker],
                         obstacle, constraint_functions)
    root.mainloop()
