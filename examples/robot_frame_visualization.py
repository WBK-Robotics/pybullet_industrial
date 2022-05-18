import os
import time
import pybullet as p
import wbk_sim as wbk
from lemniscate import build_lemniscate_path
import numpy as np

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'igus_4DOF_SV.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    p.setRealTimeSimulation(1)


    target_position = np.array([0, 0.3,0.3])
    wbk.draw_robot_frames(robot,life_time=0)
    while True:
        connection_info = p.getConnectionInfo()
        if not connection_info['isConnected']:
            break
        time.sleep(0.1)
