import os
import time
import pybullet as p
import wbk_sim as wbk
from lemniscate import build_lemniscate_path
import numpy as np

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,'robot_descriptions', 'camera.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1,[0,0,0],start_orientation)

    tool = wbk.EndeffectorTool(urdf_file2,[1.9,0,1.2],start_orientation)

    

    p.setRealTimeSimulation(1)

    target_position = [1.9,0,1.2]
    test_path = build_lemniscate_path(target_position,400,1.2,1)
    wbk.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0]) 
    while True:
        for i in range(400): 
            robot.set_endeffector_pose(test_path[:,i],target_orientation,'link6') 
            wbk.draw_coordinate_system(test_path[:,i],target_orientation)
            time.sleep(0.005) 
        tool.couple(robot)