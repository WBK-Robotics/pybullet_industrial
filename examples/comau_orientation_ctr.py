import os
import time
import pybullet as p
import wbk_sim as wbk

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_NJ290_3-0_m.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation, 'link6')

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    drawing_position = [2.4, 0, 1.2]
    wbk.draw_point(target_position, length=0.05)

    time_step = 0.01
    for i in range(300):
        target_orientation = p.getQuaternionFromEuler([0, i/100, 0])
        robot.set_endeffector_pose(target_position, target_orientation)
        wbk.draw_coordinate_system(
            drawing_position, target_orientation, life_time=10*time_step)
        time.sleep(time_step)
    for i in range(200):
        target_orientation = p.getQuaternionFromEuler([i/100, 300/100, 0])
        robot.set_endeffector_pose(target_position, target_orientation)
        wbk.draw_coordinate_system(
            drawing_position, target_orientation, life_time=10*time_step)
        time.sleep(time_step)
