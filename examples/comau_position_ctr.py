import os
import time

import numpy as np
import pybullet as p
import pybullet_industrial as pi

from lemniscate import build_lemniscate_path

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_nj290_robot.urdf')

    physics_client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    test_path = build_lemniscate_path(target_position, 400, 1.2, 1)
    pi.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    for i in range(20):
        robot.set_endeffector_pose(
            test_path[:, 0], target_orientation, 'link6')
        time.sleep(0.1)
    while True:
        for i in range(400):
            robot.set_endeffector_pose(
                test_path[:, i], target_orientation, 'link6')
            time.sleep(0.005)
            position, orientation = robot.get_endeffector_pose()
            pi.draw_coordinate_system(position, orientation)
