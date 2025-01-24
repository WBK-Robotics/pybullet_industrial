import os
import tkinter as tk
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI


def seting_up_enviroment():
    """
    Sets up the simulation environment, including paths to URDF files and the PyBullet physics simulation.
    """
    working_dir = os.path.dirname(__file__)
    urdf_robot = os.path.join(working_dir, 'robot_descriptions', 'comau_nj290_robot.urdf')
    # urdf_robot = os.path.join(working_dir, 'robot_descriptions', 'scara_control.urdf')
    # urdf_robot = os.path.join(working_dir, 'robot_descriptions', 'gantry.urdf')
    urdf_fofa = os.path.join(working_dir, 'Objects', 'FoFa', 'FoFa.urdf')

    # Comau Start Position
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])


    # # Scara Start Position
    # start_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
    # start_pos = np.array([2.0, -6.5, 0])

    # # Gantry Start Position
    # start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # start_pos = np.array([4.0, -6, 0])
    #

    # Comau Camera Position
    p.connect(p.GUI, options='--background_color_red=1 --background_color_green=1 --background_color_blue=1')
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=50.0, cameraPitch=-30,
                                 cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)

    # Scara Camera Position
    # p.resetDebugVisualizerCamera(cameraDistance=2.0/5, cameraYaw=50.0, cameraPitch=-30,
    #                              cameraTargetPosition=np.array([1.9/5, 0, 1/5]) + start_pos)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000, enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    return urdf_robot, start_pos, start_orientation


def add_box(box_pos, half_box_size):
    """
    Adds a box-shaped obstacle to the simulation.
    """
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId,
                               basePosition=box_pos, baseOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
    return box_id


if __name__ == "__main__":
    # Initialize the simulation environment
    urdf_robot, start_pos, start_orientation = seting_up_enviroment()

    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)

    # Add a box obstacle
    obstacles = []

    # Comau Obstacle
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)

    # # Scara Obstacle
    # obstacle = add_box(start_pos + [1.8/5, 0, 1.8/5], [0.5/5, 0.5/5, 0.05/5])
    # obstacles.append(obstacle)

    # Initialize CollisionChecker and PathPlanner
    collision_checker = pi.CollisionChecker(robot, obstacles)
    path_planner = pi.PathPlanner(robot, collision_checker, "RRTstar")

    # Setting up Comau
    inital_state = {'q1': -0.5, 'q2': 0, 'q3': -(np.pi/2), 'q4': -(np.pi-0.001), 'q5': -(np.pi/2), 'q6': 0}
    robot.reset_joint_position(inital_state)

    # Setting up Scara
    # inital_state = {'joint_1': -1, 'joint_2': -1, 'joint_3': -0.04}
    # robot.reset_joint_position(inital_state)

    # Create the GUI
    root = tk.Tk()
    gui = PathPlannerGUI(root, robot, path_planner, collision_checker, obstacle)
    root.mainloop()
