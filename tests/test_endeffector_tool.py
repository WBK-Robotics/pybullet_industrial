import os
import unittest
import numpy as np
import pybullet as p

import wbk_sim as wbk

dirname = os.path.dirname(__file__)
parentDir = os.path.dirname(dirname)
urdf_file1 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
urdf_file2 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'milling_head.urdf')


class TestEndeffectorTool(unittest.TestCase):
    def test_coupling(self):
        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        milling_head = wbk.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)

        first_robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
        second_robot = wbk.RobotBase(urdf_file1, [0, 1, 0], start_orientation)

        milling_head.couple(first_robot)

        pos_precision = 0.02
        ori_precision = 0.004
        within_precision = True

        for _ in range(300):
            p.stepSimulation()

        first_robot_pos, first_robot_ori = first_robot.get_endeffector_pose()
        milling_pos, milling_ori = p.getBasePositionAndOrientation(
            milling_head.urdf)
        position_error = np.linalg.norm(first_robot_pos-milling_pos)
        orientation_error = np.linalg.norm(first_robot_ori-milling_ori)
        within_precision = within_precision and (
            position_error <= pos_precision) and (orientation_error <= ori_precision)

        target_position = [1.9, 0, 1.2]
        first_robot.set_endeffector_pose(target_position)

        for _ in range(300):
            p.stepSimulation()

        first_robot_pos, first_robot_ori = first_robot.get_endeffector_pose()
        milling_pos, milling_ori = p.getBasePositionAndOrientation(
            milling_head.urdf)
        position_error = np.linalg.norm(first_robot_pos-milling_pos)
        orientation_error = np.linalg.norm(first_robot_ori-milling_ori)
        within_precision = within_precision and (
            position_error <= pos_precision) and (orientation_error <= ori_precision)

        milling_head.decouple()
        milling_head.couple(second_robot)
        for _ in range(300):
            p.stepSimulation()

        second_robot_pos, second_robot_ori = second_robot.get_endeffector_pose()
        milling_pos, milling_ori = p.getBasePositionAndOrientation(
            milling_head.urdf)
        position_error = np.linalg.norm(second_robot_pos-milling_pos)
        orientation_error = np.linalg.norm(second_robot_ori-milling_ori)
        within_precision = within_precision and (
            position_error <= pos_precision) and (orientation_error <= ori_precision)

        p.disconnect()
        self.assertTrue(within_precision)

    def test_direct_pose_movement(self):

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        milling_head = wbk.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)

        steps = 20
        target_position = [2.5, 0, 1.2]
        test_path = build_lemniscate_path(target_position, steps, 1.2, 0.8)
        target_orientation = p.getQuaternionFromEuler([0, 0, 0])

        pos_precision = 0.02
        ori_precision = 0.004
        within_precision = True
        for i in range(steps):
            target_position = test_path[:, i]
            milling_head.set_tool_pose(target_position, target_orientation)

            for _ in range(300):
                p.stepSimulation()

            current_position, current_orientation = milling_head.get_tool_pose()

            position_error = np.linalg.norm(current_position-target_position)
            orientation_error = np.linalg.norm(
                current_orientation-target_orientation)

            # discard first values where robot converges with path
            if i > 2:
                within_precision = within_precision and (
                    position_error <= pos_precision) and (orientation_error <= ori_precision)
        p.disconnect()
        self.assertTrue(within_precision)

    def test_pose_setting(self):

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
        milling_head = wbk.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)
        milling_head.couple(robot, 'link6')

        steps = 20
        target_position = [2.5, 0, 1.2]
        test_path = build_lemniscate_path(target_position, steps, 1.2, 0.8)
        target_orientation = p.getQuaternionFromEuler([0, 0, 0])

        pos_precision = 0.02
        ori_precision = 0.004
        within_precision = True
        for i in range(steps):
            target_position = test_path[:, i]
            milling_head.set_tool_pose(target_position, target_orientation)

            for _ in range(150):
                p.stepSimulation()

            current_position, current_orientation = milling_head.get_tool_pose()

            position_error = np.linalg.norm(current_position-target_position)
            orientation_error = np.linalg.norm(
                current_orientation-target_orientation)

            # discard first values where robot converges with path
            if i > 2:
                within_precision = within_precision and (
                    position_error <= pos_precision) and (orientation_error <= ori_precision)
        p.disconnect()
        self.assertTrue(within_precision)


def build_lemniscate_path(midpoint, steps, height, length):
    """Function which builds a figure 8 path

    Args:
        midpoint ([type]): [description]
        steps ([type]): [description]
        height ([type]): [description]
        length ([type]): [description]

    Returns:
        [type]: [description]
    """
    path = np.zeros((3, steps))
    path[2, :] = height
    for i in range(steps):
        path_state = 1/steps*i*2*np.pi
        path[0, i] = length * np.cos(path_state) / \
            (1+np.sin(path_state)**2)+midpoint[0]
        path[1, i] = length * np.sin(path_state) * np.cos(path_state) / \
            (1+np.sin(path_state)**2)+midpoint[1]
    return path


if __name__ == '__main__':
    unittest.main()
