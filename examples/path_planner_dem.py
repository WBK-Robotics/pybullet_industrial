import os
import tkinter as tk
from ompl import base as ob
from ompl import geometric as og
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner_gui import PathPlannerGUI


INTERPOLATE_NUM = 500  # Number of segments for interpolating the path
DEFAULT_PLANNING_TIME = 5.0  # Maximum planning time in seconds

def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase, link: chr):
    return gripper.couple(robot, link)

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


def seting_up_enviroment():
    """
    Sets up the simulation environment, including paths to URDF files and
    the PyBullet physics simulation.
    """
    working_dir = os.path.dirname(__file__)
    urdf_robot = os.path.join(working_dir, 'robot_descriptions',
                              'comau_nj290_robot.urdf')
    urdf_fofa = os.path.join(working_dir, 'Objects', 'FoFa', 'FoFa.urdf')

    urdf_gripper = os.path.join(working_dir,
                                'robot_descriptions', 'gripper_cad.urdf')
    urdf_small_cube = os.path.join(working_dir,
                              'robot_descriptions', 'cube_small.urdf')

    # Comau start position.
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])

    # Set up the GUI camera.
    p.connect(p.GUI)
            #   options='--background_color_red=1 '
            #           '--background_color_green=1 '
            #           '--background_color_blue=1')
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

    return urdf_robot, start_pos, start_orientation, urdf_gripper, urdf_small_cube


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
    urdf_robot, start_pos, start_orientation, urdf_gripper, urdf_cube_small = seting_up_enviroment()

    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
    start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])

    test_gripper = pi.Gripper(urdf_gripper, [2.7, -0.5, 1.2], start_orientation)
    position_offset = np.array([0, 0, 0])
    orientation_offset = p.getQuaternionFromEuler(np.array([-np.pi/2, 0, 0]))
    base_offset = [position_offset, orientation_offset]
    test_gripper.set_base_offset(base_offset)

    cube_small = p.loadURDF(urdf_cube_small, start_pos + [0,-2,0], useFixedBase=False)
    test_gripper.set_moving_object_offset([[0, 0, -0.1], [0, 0, 0, 0]])



    # Add a box obstacle.
    obstacles = []

    # Create a box obstacle near the robot.
    obstacle = add_box(start_pos + [1.8, 0, 1.8], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)

    # Define custom clearance query distances for each obstacle.
    # Here, we set a clearance query distance of 1.0 for the obstacle.
    clearance_obstacles = {obstacle: 1.0}


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

    # Initialize CollisionChecker with the custom clearance.

    collision_checker = pi.CollisionChecker()
    # test_gripper.match_endeffector_pose(robot)

    # test_gripper.match_moving_object(cube_small)

    collision_checker.set_safe_state()


    # Append constraint functinons
    collsion_check = [lambda: collision_checker.check_collision()]
    constraint_functions = [lambda: check_endeffector_upright(robot)]

    def clearance_objective(si): return pi.RobotPathClearanceObjective(
        si, collision_checker, 0.5)

    def path_length_objective(
        si): return ob.PathLengthOptimizationObjective(si)

    objective_weight = 1.0
    objectives = []
    # objectives.append((clearance_objective, objective_weight))
    objectives.append((path_length_objective, objective_weight))

    def rrtsharp(si): return og.RRTsharp(si)

    def rrt(si):
        rrt = og.RRT(si)
        rrt.setGoalBias(0.5)
        return rrt

    def bitstar(si): return og.BITstar(si)

    def abitstar(si): return og.ABITstar(si)

    def aitstar(si): return og.AITstar(si)

    path_planner = pi.PathPlanner(
        robot=robot,
        #endeffector=test_gripper,
        #moved_object=cube_small,
        collision_check_functions=collsion_check,
        planner_type=bitstar,
        # constraint_functions=constraint_functions,
        # objectives=objectives,
    )



    # Create the GUI for motion planning.
    root = tk.Tk()
    gui = PathPlannerGUI(root, robot, path_planner, collsion_check,
                         obstacle, constraint_functions,
                         #test_gripper,
                         #cube_small,
                         )
    root.mainloop()
