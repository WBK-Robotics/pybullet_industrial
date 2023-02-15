import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np


if __name__ == "__main__":
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot.urdf')
    endeffector_file = os.path.join(
        file_directory, 'urdf', 'welding_torch.urdf')

    physics_client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = pi.RobotBase(sdmbot_urdf_file, [0, 0, 0], start_orientation)
    endeffector = pi.EndeffectorTool(
        endeffector_file, [0, 0, 0], start_orientation)

    test_tool = pi.EndeffectorTool(
        endeffector_file, [0, 0, 0], start_orientation)
    test_tool.set_tool_pose([0, 0, 0.63], [0, 0, 0, 1])
    endeffector.couple(robot)

    position = np.array([0, 0, 0.63])

    while True:
        endeffector.set_tool_pose(position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(0.01)
