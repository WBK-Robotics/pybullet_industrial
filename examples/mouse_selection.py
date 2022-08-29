import os

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file2 = os.path.join(
        dirname, 'robot_descriptions', 'kuka_robot.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [-2, 0, 0], start_orientation)
    start_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
    robot = pi.RobotBase(urdf_file2, [2, 0, 0], start_orientation)

    p.setRealTimeSimulation(1)

    colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
    currentColor = 0

    p.getCameraImage(64, 64, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    while True:
        objectUid, object_index = pi.get_object_id_from_mouse()
        if (objectUid >= 0):
            p.changeVisualShape(objectUid, object_index,
                                rgbaColor=colors[currentColor])
            currentColor += 1
            if currentColor >= len(colors):
                currentColor = 0
