import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname, 'robot_descriptions', 'camera.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    start_orientation = p.getQuaternionFromEuler([np.pi/2, -np.pi/4, np.pi])
    camera_parameters = {'width': 480, 'height': 240, 'fov': 60,
                         'aspect ratio': 1, 'near plane distance': 0.01, 'far plane distance': 100}
    camera = pi.Camera(urdf_file2, [0, -2, 1.2],
                        start_orientation, camera_parameters)

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    time_step = 0.001
    while True:
        for i in range(300):
            target_orientation = p.getQuaternionFromEuler([0, i/100, 0])
            robot.set_endeffector_pose(target_position, target_orientation)
            camera.get_image()
            time.sleep(time_step)

        if not camera.is_coupled():
            camera.couple(robot, 'link6')
        else:
            camera.decouple()
