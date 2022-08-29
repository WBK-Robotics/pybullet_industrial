import os
import time

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', '3d_printing_head.urdf')

    urdf_file2 = os.path.join(
        dirname, 'robot_descriptions', 'comau_nj290_robot.urdf')

    physics_client = p.connect(
        p.GUI, options='--background_color_red=0 ' +
                       '--background_color_green=109/255 ' +
                       '--background_color_blue=94/255')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot1 = pi.RobotBase(urdf_file1, [0, -1, 0], start_orientation)
    robot2 = pi.RobotBase(urdf_file2, [0, 1, 0], start_orientation)

    p.setRealTimeSimulation(1)

    target_position = np.array([0, 0.3, 0.3])
    pi.draw_robot_frames(robot1, life_time=0)
    pi.draw_robot_frames(robot2, life_time=0, text_size=2,length=0.4)
    while True:
        connection_info = p.getConnectionInfo()
        if not connection_info['isConnected']:
            break
        time.sleep(0.1)
