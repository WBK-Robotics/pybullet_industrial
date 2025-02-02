import os
import unittest
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi


def seting_up_enviroment():
    """
    Sets up the simulation environment, including URDF file paths and the
    PyBullet physics simulation. This version uses DIRECT mode for testing.

    Returns:
        tuple: (urdf_robot, start_pos, start_orientation)
    """
    working_dir = os.path.dirname(__file__)
    urdf_robot = os.path.join(working_dir, "..", "examples", 'robot_descriptions',
                              'comau_nj290_robot.urdf')
    # Define the robot's start position and orientation.
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    start_pos = np.array([2.0, -6.5, 0])

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.setPhysicsEngineParameter(numSolverIterations=5000, enableFileCaching=0)

    # Optionally, load a fixed object (e.g., a FoFa object) into the environment.
    urdf_fofa = os.path.join(working_dir, "..", "examples", 'Objects', 'FoFa', 'FoFa.urdf')
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)

    return urdf_robot, start_pos, start_orientation


def add_box(box_pos, half_box_size):
    """
    Adds a box-shaped obstacle to the simulation.

    Args:
        box_pos (list or np.array): The position of the box.
        half_box_size (list): Half of the dimensions (x, y, z) of the box.

    Returns:
        int: The PyBullet ID for the created box.
    """
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=colBoxId,
                               basePosition=box_pos,
                               baseOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
    return box_id


class TestCollisionChecker(unittest.TestCase):
    """Tests for the CollisionChecker class in an environment set up like the
    example code.
    """
    def setUp(self):
        """Set up the PyBullet simulation, robot, and obstacles."""
        # Set up the simulation environment.
        self.urdf_robot, self.start_pos, self.start_orientation = \
            seting_up_enviroment()

        # Create the robot using the RobotBase class.
        self.robot = pi.RobotBase(self.urdf_robot,
                                  self.start_pos,
                                  self.start_orientation)

        # Add a box obstacle that is far from the robot.
        self.obstacles = []
        obstacle_far = add_box(self.start_pos + np.array([1.8, 0, 1.8]),
                                [0.5, 0.5, 0.05])
        self.obstacles.append(obstacle_far)

        # Initialize the CollisionChecker with the far obstacle.
        self.collision_checker = pi.CollisionChecker(self.robot,
                                                       self.obstacles,
                                                       self_collisions=False,
                                                       max_distance=0.05)

    def tearDown(self):
        """Disconnect the PyBullet simulation."""
        p.disconnect()

    def test_no_collision(self):
        """
        Test that no collision is detected when the obstacle is far away.
        """
        # Since the obstacle is far away, check_collision() should return True.
        self.assertTrue(self.collision_checker.check_collision())

    def test_collision(self):
        """
        Test that a collision is detected when an obstacle overlaps the robot.
        """
        # Create an obstacle that overlaps the robot's base.
        obstacle_overlap = add_box(self.start_pos + np.array([0, 0, 0]),
                                   [1.0, 1.0, 1.0])
        # Update the CollisionChecker with the overlapping obstacle.
        self.collision_checker.set_obstacles([obstacle_overlap])
        # Now a collision should be detected (check_collision() returns False).
        self.assertFalse(self.collision_checker.check_collision())


if __name__ == '__main__':
    unittest.main()
