import os
import unittest

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


class TestGrippers(unittest.TestCase):
    def test_grippers(self):
        dirname = os.path.dirname(__file__)
        parent_dir = os.path.dirname(dirname)
        urdf_file1 = os.path.join(parent_dir, 'examples',
                                  'robot_descriptions', 'comau_nj290_robot.urdf')
        urdf_file2 = os.path.join(parent_dir, 'examples',
                                  'robot_descriptions', 'gripper_cad.urdf')
        urdf_file4 = os.path.join(parent_dir, 'examples',
                                  'robot_descriptions', 'screwDriver.urdf')
        urdf_file3 = os.path.join(parent_dir, 'examples',
                                  'robot_descriptions', 'cube_small.urdf')

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=10000)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0, 0, -10)
        start_pos = np.array([2.0, -6.5, 0])
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, start_pos, start_orientation)

        start_pos2 = np.array([6.25, -6.5, 0])
        start_orientation2 = p.getQuaternionFromEuler([0, 0, np.pi])
        robot2 = pi.RobotBase(urdf_file1, start_pos2, start_orientation2)

        robot.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
        robot2.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
        for _ in range(100):
            p.stepSimulation()

        p.loadURDF("cube.urdf", np.array(
            [2.125, 0, 0.5]) + start_pos, useFixedBase=True)

        gripper = pi.Gripper(
            urdf_file2, [1.9, 0, 1.2] + start_pos, start_orientation)
        gripper.couple(robot, 'link6')

        start_orientation_sg = p.getQuaternionFromEuler([0, 0, np.pi / 2])
        suction_gripper = pi.SuctionGripper(
            urdf_file4, [-1.9, 0, 5] + start_pos2, start_orientation, suction_links=["tcp"])

        suction_gripper.couple(robot2, 'link6')

        p.resetDebugVisualizerCamera(cameraDistance=4.8, cameraYaw=50.0, cameraPitch=-30,
                                     cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)

        start_orientation_gr = p.getQuaternionFromEuler([-np.pi, 0, 0])

        safepoint1 = start_pos + [1.9, 0.5, 2.0]
        safepoint2 = start_pos2 + [-1.9, -0.5, 2.0]
        grippoint1 = [1.9, -0.25, 1.05] + start_pos
        grippoint1_2 = grippoint1 + [0, 0, 0.1]
        droppoint1 = [-1.9, 0.25, 1.15] + start_pos2
        grippoint2 = droppoint1.copy() + [0, 0, -0.1105]
        grippoint2_2 = grippoint2 + [0, 0, 0.1]
        droppoint2 = grippoint1.copy() + [0, 0, 0.1]

        for _ in range(25):
            p.stepSimulation()

        for _ in range(8):
            gripper.set_tool_pose(safepoint1, start_orientation_gr)
            gripper.actuate(0.0)
            suction_gripper.set_tool_pose(safepoint2, start_orientation_sg)
            for _ in range(20):
                p.stepSimulation()

        minicube = p.loadURDF(urdf_file3, grippoint1, useFixedBase=False)
        for _ in range(20):
            p.stepSimulation()

        pos_precision = 0.02
        within_precision = True

        path = pi.linear_interpolation(np.array(safepoint1),
                                       np.array(grippoint1_2), 10)
        path.orientations = np.transpose(
            [start_orientation_gr] * len(path.orientations[0]))
        move_along_path(gripper, path, start_orientation_gr)
        path = pi.linear_interpolation(
            np.array(grippoint1_2), np.array(grippoint1), 10)
        path.orientations = np.transpose(
            [start_orientation_gr] * len(path.orientations[0]))
        move_along_path(gripper, path)
        for _ in range(25):
            p.stepSimulation()
        gripper.actuate(1.0)
        for _ in range(25):
            p.stepSimulation()
        path = pi.linear_interpolation(np.array(grippoint1),
                                       np.array(grippoint1_2), 10)
        path.orientations = np.transpose(
            [start_orientation_gr] * len(path.orientations[0]))
        move_along_path(gripper, path)
        path = pi.linear_interpolation(
            np.array(grippoint1_2), np.array(droppoint1), 10)
        path.orientations = np.transpose(
            [start_orientation_gr] * len(path.orientations[0]))
        move_along_path(gripper, path)
        for _ in range(25):
            p.stepSimulation()
        gripper.actuate(0.0)
        for _ in range(25):
            p.stepSimulation()
        path = pi.linear_interpolation(np.array(droppoint1),
                                       np.array(safepoint1), 10)
        path.orientations = np.transpose(
            [start_orientation_gr] * len(path.orientations[0]))
        move_along_path(gripper, path)

        current_position, _ = p.getBasePositionAndOrientation(minicube)
        position_error = np.linalg.norm(current_position - grippoint2)
        within_precision = within_precision and (
            position_error <= pos_precision)

        path = pi.linear_interpolation(np.array(safepoint2),
                                       np.array(grippoint2_2), 10)
        path.orientations = np.transpose(
            [start_orientation_sg] * len(path.orientations[0]))
        move_along_path(suction_gripper, path, start_orientation_sg)
        path = pi.linear_interpolation(
            np.array(grippoint2_2), np.array(grippoint2), 10)
        path.orientations = np.transpose(
            [start_orientation_sg] * len(path.orientations[0]))
        move_along_path(suction_gripper, path)
        for _ in range(25):
            p.stepSimulation()
        suction_gripper.activate(0.001)
        for _ in range(25):
            p.stepSimulation()
        path = pi.linear_interpolation(np.array(grippoint2),
                                       np.array(grippoint2_2), 10)
        path.orientations = np.transpose(
            [start_orientation_sg] * len(path.orientations[0]))
        move_along_path(suction_gripper, path)
        path = pi.linear_interpolation(
            np.array(grippoint2_2), np.array(droppoint2), 10)
        path.orientations = np.transpose(
            [start_orientation_sg] * len(path.orientations[0]))
        move_along_path(suction_gripper, path)
        for _ in range(25):
            p.stepSimulation()
        suction_gripper.deactivate()
        for _ in range(25):
            p.stepSimulation()
        path = pi.linear_interpolation(np.array(droppoint2),
                                       np.array(safepoint2), 10)
        path.orientations = np.transpose(
            [start_orientation_sg] * len(path.orientations[0]))
        move_along_path(suction_gripper, path)

        current_position, _ = p.getBasePositionAndOrientation(minicube)
        position_error = np.linalg.norm(current_position - grippoint1)
        within_precision = within_precision and (
            position_error <= pos_precision)
        p.disconnect()
        self.assertTrue(within_precision)


def move_along_path(endeffector: pi.EndeffectorTool, path: pi.ToolPath, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (pi.EndeffectorTool): Endeffector to be moved.
        path (pi.ToolPath): Array of points defining the path.

    """
    for positions, orientations, tool_path in path:
        endeffector.set_tool_pose(positions, orientations)
        for _ in range(10):
            p.stepSimulation()
    if stop:
        for _ in range(100):
            p.stepSimulation()


if __name__ == '__main__':
    unittest.main()
