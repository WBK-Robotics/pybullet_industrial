import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner import PathPlanner
from robot_base import RobotBase


if __name__ == "__main__":

    # Setting up the paths
    working_dir = os.path.dirname(__file__)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    urdf_robot = os.path.join(
        working_dir, 'robot_descriptions', 'comau_nj290_robot.urdf')

    urdf_fofa = os.path.join(
        working_dir, 'Objects', 'FoFa', 'FoFa.urdf')

    # Setting up the simulation
    start_pos = np.array([2.0, -6.5, 0])
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
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    # Setting up robot position 
    robot = RobotBase(urdf_robot, start_pos, start_orientation)

    # Setting robot configuration (fast way)
    joint_position = [-6, 0, -1, 0, 0, 0]
    robot.set_robot(joint_position)

    # Finding out more about collision
    path_planner = PathPlanner(robot)
    path_planner.collision_report()

   

    g_code_logger = pi.GCodeLogger(robot)
    g_code_logger.update_g_code()
    rounded_g_code = {key: round(
        value, 4) for key, value in g_code_logger.g_code_joint_position[0].items()}
    print(rounded_g_code)
    

