
"""This example shows how to use the RobotBase class endeffector interface can be used
   to control the orientation of a robot endeffector.
"""

import os
import time

import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'kuka_robot.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation, 'link6')

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    drawing_position = [2.4, 0, 1.2]
    pi.draw_point(target_position, length=0.05)

    time_step = 0.01
    for i in range(300):
        target_orientation = p.getQuaternionFromEuler([i/100, 0, 0])
        robot.set_endeffector_pose(target_position, target_orientation)
        pi.draw_coordinate_system(
            drawing_position, target_orientation, life_time=10*time_step)
        time.sleep(time_step)
    for i in range(300):
        target_orientation = p.getQuaternionFromEuler([300/100, 0, i/100])
        robot.set_endeffector_pose(target_position, target_orientation)
        pi.draw_coordinate_system(
            drawing_position, target_orientation, life_time=10*time_step)
        time.sleep(time_step)
