import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner import PathPlanner


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
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    # Setting up robot position
    robot = pi.RobotBase(urdf_robot, start_pos, start_orientation)
    robot.set_joint_position(
        {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
    for _ in range(100):
        p.stepSimulation()
    new_point = robot.get_endeffector_pose()[0]
    robot.set_endeffector_pose(
        new_point, p.getQuaternionFromEuler([0, 0, 0]))
    for _ in range(100):
        p.stepSimulation()

    g_code_logger = pi.GCodeLogger(robot)
    g_code_logger.update_g_code()
    print("wait")
