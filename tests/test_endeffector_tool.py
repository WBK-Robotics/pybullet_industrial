import os
import unittest
import numpy as np
import pybullet as p

import pybullet_industrial as pi

dirname = os.path.dirname(__file__)
parentDir = os.path.dirname(dirname)
urdf_file1 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
urdf_file2 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'milling_head.urdf')

steps = 20
center_position = [2.5, 0, 1.2]
test_path = pi.build_box_path(
    center_position, [1.2, 0.8], 0.01, [0, 0, 0, 1], steps)


def spawn_pendulum(start_position):
    urdf_path = os.path.join(parentDir, 'examples',
                             'robot_descriptions', 'pendulum.urdf')
    pendulum = pi.EndeffectorTool(
        urdf_path, start_position, [0, 0, 0, 1])

    p.resetJointState(pendulum.urdf, 0, targetValue=0.5)
    return pendulum


class TestEndeffectorTool(unittest.TestCase):
    def test_coupling(self):
        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        milling_head = pi.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)

        first_robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
        second_robot = pi.RobotBase(urdf_file1, [0, 1, 0], start_orientation)

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
        milling_head = pi.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)

        pos_precision = 0.02
        ori_precision = 0.004
        within_precision = True
        for target_position, target_orientation, _ in test_path:
            milling_head.set_tool_pose(target_position, target_orientation)

            for _ in range(300):
                p.stepSimulation()

            current_position, current_orientation = milling_head.get_tool_pose()

            position_error = np.linalg.norm(current_position-target_position)
            orientation_error = np.linalg.norm(
                current_orientation-target_orientation)
            within_precision = within_precision and (
                position_error <= pos_precision) and (orientation_error <= ori_precision)
        p.disconnect()
        self.assertTrue(within_precision)

    def test_pose_setting(self):

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
        milling_head = pi.EndeffectorTool(
            urdf_file2, [1.9, 0, 1.2], start_orientation)
        milling_head.couple(robot, 'link6')

        pos_precision = 0.02
        ori_precision = 0.006
        within_precision = True

        for _ in range(20):
            milling_head.set_tool_pose(*test_path.get_start_pose())
            for _ in range(50):
                p.stepSimulation()

        for target_position, target_orientation, _ in test_path:
            milling_head.set_tool_pose(target_position, target_orientation)

            for _ in range(150):
                p.stepSimulation()

            current_position, current_orientation = milling_head.get_tool_pose()

            position_error = np.linalg.norm(current_position-target_position)
            orientation_error = np.linalg.norm(
                current_orientation-target_orientation)

            # discard first values where robot converges with path
            within_precision = within_precision and (
                position_error <= pos_precision) and (orientation_error <= ori_precision)
        p.disconnect()
        self.assertTrue(within_precision)

    def test_external_force_setting(self):
        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=15000)

        pendulum1 = spawn_pendulum([0, 0, 0])
        pendulum2 = spawn_pendulum([0, 0.6, 0])
        pendulum3 = spawn_pendulum([0, 1.2, 0])
        pendulum4 = spawn_pendulum([0, 1.8, 0])

        p.setTimeStep(0.5)
        steps = 10
        for _ in range(steps):
            pendulum1.apply_tcp_force([-10, 0, 20.0], world_coordinates=True)
            pendulum2.apply_tcp_force([0, 1, 0])
            pendulum3.apply_tcp_force(
                [-1, 0, 0.0], world_coordinates=False)
            pendulum4.apply_tcp_torque([00, 1, 00])

            p.stepSimulation()

            position1, _ = pendulum1.get_tool_pose()
            position2, _ = pendulum2.get_tool_pose()
            pendulum_state3 = p.getLinkState(
                pendulum3.urdf, 0, computeLinkVelocity=1)
            pendulum_state4 = p.getLinkState(
                pendulum4.urdf, 0, computeLinkVelocity=1)

        pendulum1_upgright = position1[2]-0.5 <= 0.001

        init_pendulum_position = np.array([0.23971271, 0.60000369, 0.43879131])
        pendulum2_unmoved = np.linalg.norm(
            position2-init_pendulum_position) <= 0.001

        pendulum3_rotating_anti_clockwise = pendulum_state3[7][1] < 0
        pendulum4_rotating_clockwise = pendulum_state4[7][1] > 0

        p.disconnect()
        self.assertTrue(pendulum1_upgright and pendulum2_unmoved and
                        pendulum3_rotating_anti_clockwise and pendulum4_rotating_clockwise)


if __name__ == '__main__':
    unittest.main()
