import os
import unittest
import numpy as np
import pybullet as p

import pybullet_industrial as pi


class TestRobotBase(unittest.TestCase):
    def test_joint_interface(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file1 = os.path.join(
            parentDir, 'examples', 'robot_descriptions', 'comau_nj290_robot.urdf')

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        precision = 10**-4
        within_precision = True
        for i in range(100):
            oscillation = np.sin(i/20)
            target_state = {'q1': oscillation, 'q2': oscillation, 'q3': oscillation *
                            0.4-0.4, 'q4': oscillation, 'q5': oscillation, 'q6': oscillation}
            robot.set_joint_position(target_state)
            for _ in range(100):
                p.stepSimulation()

            actual_state = robot.get_joint_state()
            q1_error = target_state['q1']-actual_state['q1']['position']
            q2_error = target_state['q2']-actual_state['q2']['position']
            q3_error = target_state['q3']-actual_state['q3']['position']
            q4_error = target_state['q4']-actual_state['q4']['position']
            q5_error = target_state['q5']-actual_state['q5']['position']
            q6_error = target_state['q6']-actual_state['q6']['position']

            within_precision = within_precision and ((q1_error <= precision) and
                                                     (q2_error <= precision) and
                                                     (q3_error <= precision) and
                                                     (q4_error <= precision) and
                                                     (q5_error <= precision) and
                                                     (q6_error <= precision))
        p.disconnect()
        self.assertTrue(within_precision)

    def test_position_interface_igus(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file1 = os.path.join(
            parentDir, 'examples', 'robot_descriptions', 'igus_4dof_robot.urdf')

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(
            urdf_file1, [0, 0, 0], start_orientation, 'link4')

        precision = 0.02
        within_precision = True
        for i in range(10):
            target_pose = np.array([i/400+0.2, -i/400-0.2, 0.3])
            robot.set_endeffector_pose(target_pose)
            for _ in range(200):
                p.stepSimulation()
            current_pose, _ = robot.get_endeffector_pose()
            position_error = np.linalg.norm(current_pose-target_pose)
            within_precision = within_precision and (
                position_error <= precision)
        p.disconnect()
        self.assertTrue(within_precision)

    def test_pose_interface_comau(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file1 = os.path.join(
            parentDir, 'examples', 'robot_descriptions', 'comau_nj290_robot.urdf')

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        pos_precision = 0.02
        ori_precision = 0.001
        within_precision = True
        for i in range(16):
            target_orientation = p.getQuaternionFromEuler([0, i/10, 0])
            target_position = [1.9, 0, 1.2]

            for _ in range(1000):
                robot.set_endeffector_pose(
                    target_position, target_orientation, 'link6')
                p.stepSimulation()

            current_position, current_orientation = robot.get_endeffector_pose(
                'link6')

            position_error = np.linalg.norm(current_position-target_position)
            orientation_error = np.linalg.norm(
                current_orientation-target_orientation)

            # disregard first 4 measurments because inv kin solver first converges
            if i >= 5:
                within_precision = within_precision and (
                    position_error <= pos_precision) and (orientation_error <= ori_precision)
        p.disconnect()
        self.assertTrue(within_precision)

    def test_reset_functions(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file = os.path.join(
            parentDir, 'examples', 'robot_descriptions',
            'comau_nj290_robot.urdf'
        )
        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file, [0, 0, 0], start_orientation)

        precision = 1e-6
        within_precision = True
        joint_names, _ = robot.get_moveable_joints()
        lower_limits, upper_limits = robot.get_joint_limits()

        for i in range(50):
            oscillation = np.sin(i / 20)
            target_state = {
                name: np.clip(
                    oscillation * (0.4 - 0.1 * idx),
                    lower_limits[name],
                    upper_limits[name]
                )
                for idx, name in enumerate(joint_names)
            }
            robot.reset_joint_position(target_state)
            actual_positions = robot.get_joint_position()
            for name in joint_names:
                error = abs(target_state[name] - actual_positions[name])
                within_precision = within_precision and (error <= precision)

        self.assertTrue(
            within_precision,
            msg="reset_joint_position did not set joints precisely."
        )

        invalid_target = {'invalid_joint': 0.0}
        with self.assertRaises(KeyError):
            robot.reset_joint_position(invalid_target)

        invalid_target = {}
        for idx, name in enumerate(joint_names):
            if idx == 0:
                invalid_target[name] = lower_limits[name] - 1.0
            else:
                invalid_target[name] = np.clip(
                    0.0, lower_limits[name], upper_limits[name]
                )
        with self.assertRaises(ValueError):
            robot.reset_joint_position(invalid_target)

        pos_precision = 0.02
        within_precision = True
        for i in range(10):
            target_pose = np.array([i / 400 + 1.0, -i / 400, 1.2])
            robot.reset_endeffector_pose(
                target_position=target_pose, endeffector_name='link6'
            )
            current_pose, _ = robot.get_endeffector_pose('link6')
            position_error = np.linalg.norm(current_pose - target_pose)
            within_precision = within_precision and (position_error <= pos_precision)

        try:
            target_pose_default = np.array([0.7, 0.0, 0.8])
            target_orientation_default = p.getQuaternionFromEuler([0, 0, 0])
            robot.reset_endeffector_pose(
                target_position=target_pose_default,
                target_orientation=target_orientation_default)
        except Exception as e:
            self.fail(
                "reset_endeffector_pose (default branch) raised an unexpected "
                "exception: " + str(e)
            )
        current_pose_default, _ = robot.get_endeffector_pose()
        self.assertIsInstance(current_pose_default, np.ndarray)

        p.disconnect()
        self.assertTrue(
            within_precision,
            msg="reset_endeffector_pose did not reach target positions "
                "accurately."
        )


if __name__ == '__main__':
    unittest.main()
