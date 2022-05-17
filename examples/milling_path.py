import os
import time
import pybullet as p
import wbk_sim as wbk
from lemniscate import build_lemniscate_path

if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    milling_head = wbk.EndeffectorTool(
        urdf_file2, [1.9, 0, 1.2], start_orientation)
    milling_head.couple(robot, 'link6')

    target_position = [2.3, 0, 1.2]
    test_path = build_lemniscate_path(target_position, 200, 1.2, 0.8)
    wbk.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    p.setRealTimeSimulation(1)
    while True:
        for i in range(400):
            milling_head.set_tool_pose(test_path[:, i], target_orientation)
            wbk.draw_coordinate_system(test_path[:, i], target_orientation)
            time.sleep(0.005)
