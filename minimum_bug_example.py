import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np


def accurate_calculate_inverse_kinematics(robot, target_pos, target_ori, threshold, max_iter):
    close_enough = False
    iter_num = 0
    dist2 = 1e30

    while (not close_enough and iter_num < max_iter):
        p.stepSimulation()
        robot.set_endeffector_pose(target_pos, target_ori)

        new_pos = robot.get_endeffector_pose()[0]
        diff = [target_pos[0] - new_pos[0], target_pos[1] -
                new_pos[1], target_pos[2] - new_pos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        close_enough = (dist2 < threshold)
        iter_num += 1

    print("Num iter: " + str(iter_num) + " threshold: " + str(dist2))
    if iter_num <= 10:
        print("joint state: " + str(robot.get_joint_state()))
    return


def accurate_tool_pose(end_effector, robot, target_position, target_orientation, threshold=0.001, max_iter=10000):
    tcp_translation_inv, tcp_rotation_inv = p.invertTransform(
        end_effector._tcp_translation, end_effector._tcp_rotation)
    adj_target_position, adj_target_orientation = p.multiplyTransforms(target_position, target_orientation,
                                                                       tcp_translation_inv, tcp_rotation_inv)

    accurate_calculate_inverse_kinematics(
        robot, adj_target_position, adj_target_orientation, threshold, max_iter)


if __name__ == "__main__":
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot.urdf')
    endeffector_file = os.path.join(
        file_directory, 'examples', 'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=10000)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = pi.RobotBase(sdmbot_urdf_file, [0, 0, 0], start_orientation)
    robot_id = robot.urdf
    endeffector_index = robot._default_endeffector_id
    endeffector = pi.EndeffectorTool(
        endeffector_file, [0, 0, 0], start_orientation)

    test_tool = pi.EndeffectorTool(
        endeffector_file, [0, 0, 0], start_orientation)
    test_tool.set_tool_pose([0, 0, 0.63], [0, 0, 0, 1])
    endeffector.couple(robot)

    desired_position = np.array([0, 0, 0.63])

    initial_joint_state = {
        'shoulder_pan_joint': -1.841800468078536,
        'shoulder_lift_joint': -0.07888051657301794,
        'elbow_joint': -0.7281539656326238,
        'wrist_1_joint': 0.8078408186441666,
        'wrist_2_joint': -3.474046522564269,
        'wrist_3_joint': -1.5708191849156867
        }

    for _ in range(100):
        robot.set_joint_position(initial_joint_state)
        p.stepSimulation()

    actual_position, _ = endeffector.get_tool_pose()
    print(actual_position, desired_position)
    while True:

        accurate_tool_pose(endeffector, robot, desired_position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(0.01)
