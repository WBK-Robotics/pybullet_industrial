"""A simple class that encapsulates mobile robots."""
import numpy as np
import pybullet as p
import unittest
import os
import pybullet_data
import pybullet_industrial as pi


class TestAGVRobot(unittest.TestCase):

    def setUp(self):
        p.connect(p.DIRECT)
        self.start_position = np.array([0, 0, 0])
        self.start_orientation = np.array([0, 0, 0,1])
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'diff_drive_agv.urdf')
        self.robot = pi.AGVRobot(urdf_file, self.start_position, self.start_orientation)

    def tearDown(self):
        p.disconnect()

    def test_get_and_set_world_state(self):
        # Set a new world state for the robot
        new_position = np.array([1, 1, 0])
        new_orientation = np.array([0, 0, 0, 1])
        self.robot.set_world_state(new_position, new_orientation)
        # Get the world state of the robot
        position, orientation = self.robot.get_world_state()
        self.assertTrue(np.allclose(position, new_position))
        self.assertTrue(np.allclose(orientation, new_orientation))

    def test_calculate_position_error(self):

        # set the robot to the origin with zero orientation
        self.robot.set_world_state(np.array([0, 0, 0]), np.array([0, 0, 0, 1]))

        # set the target position to (1,1,0) and the target orientation to zero
        target_position = np.array([1, 1, 0])
        target_orientation = np.array([0, 0, 0, 1])

        target_states= [[np.array([1, 1, 0]), np.array([0, 0, 0, 1])],
                        [np.array([1,-1, 0]), p.getQuaternionFromEuler([0, 0, np.pi/2])],
                        [np.array([-1, 0, 0]), np.array([0, 0, 0, 1])]]

        correct_results = [[np.sqrt(2), np.pi/4, 0],
                            [np.sqrt(2), -np.pi/4, - np.pi/2],
                            [1, np.pi, 0]]

        for target_state, correct_result in zip(target_states, correct_results):
            target_position = target_state[0]
            target_orientation = target_state[1]
            self.robot.set_target_pose(target_position, target_orientation)

            # calculate the position error
            distance, angle, target_angle_error = self.robot._calculate_position_error()

            # check if the position error is correct
            self.assertAlmostEqual(distance, correct_result[0])
            self.assertAlmostEqual(angle, correct_result[1])
            self.assertAlmostEqual(target_angle_error, correct_result[2])



class TestDiffDriveAGV(unittest.TestCase):

    def setUp(self):

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.81)

        self.start_position = np.array([0, 0, 0.4])
        self.start_orientation = np.array([0, 0, 0,1])
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'diff_drive_agv.urdf')
        self.robot = pi.DiffDriveAGV(urdf_file, self.start_position, self.start_orientation,
                                  "left_wheel_joint", "right_wheel_joint",
                                  {"wheel_radius": 0.1,
                                   "track_width": 0.5,
                                   "max_linear_velocity": 1,
                                   "max_angular_velocity": 1})

    def tearDown(self) -> None:
        p.disconnect()

    def test_calculate_wheel_commands(self):

        wheel_commands = self.robot._calculate_wheel_comands([1, 0])
        self.assertTrue(np.allclose(wheel_commands, [10, 10]))


        wheel_commands = self.robot._calculate_wheel_comands([0, 1])
        self.assertTrue(np.allclose(wheel_commands, [-2.5, 2.5]))


        wheel_commands = self.robot._calculate_wheel_comands([1, 1])
        self.assertTrue(np.allclose(wheel_commands, [7.5, 12.5]))

    def test_set_velocity(self):

            #self.robot.set_velocity([1, 0])
            for _ in range(80):
                self.robot.set_velocity([1, 0])
                p.stepSimulation()
            # get the body velocity of the robot
            body_velocity = p.getBaseVelocity(self.robot.urdf)[0]
            self.assertTrue(np.allclose(body_velocity, [1, 0, 0], atol=0.1))

    def test_standard_position_controller(self):
        # set the robot to the origin with zero orientation
        self.robot.set_world_state(np.array([0, 0, 0.5]), np.array([0, 0, 0, 1]))
        for _ in range(10):
            p.stepSimulation()

        # set the target position to (1,1,0) and the target orientation to zero
        target_position = np.array([2, 0, 0])
        target_orientation = np.array([0, 0, 0, 1])
        self.robot.set_target_pose(target_position, target_orientation)
        for _ in range(100):
            self.robot.update_position_loop()
            for _ in range(50):
                p.stepSimulation()
        # calculate the position error
        distance, angle, target_angle_error = self.robot._calculate_position_error()

        # check if the position error converges to zero
        self.assertAlmostEqual(distance, 0, delta=0.05)
        self.assertAlmostEqual(target_angle_error, 0, delta=0.1)

        # set the target position to (1,-1,0) and the target orientation to pi/2
        target_position = np.array([3, -2, 0])
        target_orientation = p.getQuaternionFromEuler([0, 0, np.pi/2])
        self.robot.set_target_pose(target_position, target_orientation)
        for _ in range(100):
            self.robot.update_position_loop()
            for _ in range(50):
                p.stepSimulation()

        # calculate the position error
        distance, angle, target_angle_error = self.robot._calculate_position_error()

        # check if the position error is correct
        self.assertAlmostEqual(distance, 0, delta=0.05)




if __name__ == "__main__":
    unittest.main()