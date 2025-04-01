import os
import unittest
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi


def setup_environment():
    """
    Sets up the PyBullet simulation environment with required
    parameters and returns the robot URDF path, initial position,
    and initial orientation.

    Returns:
        tuple: (urdf_robot, start_pos, start_orient)
    """
    p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    dirname = os.path.dirname(__file__)
    parent_dir = os.path.dirname(dirname)
    urdf_robot = os.path.join(parent_dir, 'examples',
                              'robot_descriptions',
                              'comau_nj290_robot.urdf')
    start_pos = np.array([0, 0, 0])
    start_orient = p.getQuaternionFromEuler([0, 0, 0])
    return urdf_robot, start_pos, start_orient


def create_collision_box(start_pos):
    """
    Creates a box with collision and visual shapes at the given
    start position.

    Args:
        start_pos (list or tuple): The position for the box.

    Returns:
        int: The unique identifier of the created box.
    """
    box_shape = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[0.5, 0.5, 0.5])
    box_visual = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[0.5, 0.5, 0.5])
    box_id = p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=box_shape,
                               baseVisualShapeIndex=box_visual,
                               basePosition=start_pos)
    return box_id


def create_visual_box(start_pos):
    """
    Creates a box with visual shape at the given start position.

    Args:
        start_pos (list or tuple): The position for the box.

    Returns:
        int: The unique identifier of the created box.
    """
    box_visual = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[0.5, 0.5, 0.5])
    box_id = p.createMultiBody(baseMass=0,
                               baseVisualShapeIndex=box_visual,
                               basePosition=start_pos)
    return box_id


class TestCollisionChecker(unittest.TestCase):
    """
    A suite of tests that verify the functionality of the
    CollisionChecker class. Tests cover:

      - Body management (adding, removing, static conversion)
      - Internal and external collision detection
      - Ignoring collisions (both internal and external)
      - Distance computations in controlled scenarios
    """

    def setUp(self):
        (self.urdf_robot, self.start_pos,
         self.start_orient) = setup_environment()

    def tearDown(self):
        p.disconnect()

    def test_collision_pairs(self):
        """
        Verifies that the CollisionChecker correctly tracks bodies,
        updates collision pairs, and reflects changes when a robot is
        converted to static, removed, and re-added.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        robot_b = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        collision_box = create_collision_box([0, 0, 0])
        visual_box = create_visual_box([0, 0, 0])
        body_ids = [robot_a.urdf, robot_b.urdf, visual_box]
        col_checker = pi.CollisionChecker(body_ids=body_ids)

        # Check that both robot entries are present.
        # Visual box should not be included.
        self.assertEqual(len(col_checker._bodies_information),
                         len(body_ids)-1)
        for body in col_checker._bodies_information:
            self.assertEqual(body['body_type'], 'robot')

        # Verify that internal collision pairs exist for each robot.
        expected_internal = len([b for b in col_checker._bodies_information
                                 if b['body_type'] == 'robot'])
        self.assertEqual(len(col_checker._internal_collision_pairs),
                         expected_internal)
        # External collision pairs should cover one body pair.
        self.assertEqual(len(col_checker._external_collision_pairs), 1)

        # Convert the second robot to a static body.
        col_checker.make_robot_static(robot_b.urdf)
        entry_robot_b = [body for body in col_checker._bodies_information
                         if body['body_id'] == robot_b.urdf][0]
        self.assertEqual(entry_robot_b['body_type'], 'static_body')
        expected_internal = len([b for b in col_checker._bodies_information
                                 if b['body_type'] == 'robot'])
        self.assertEqual(len(col_checker._internal_collision_pairs),
                         expected_internal)

        # Remove the second robot and verify that updates occur.
        col_checker.remove_body_id(robot_b.urdf)
        self.assertEqual(len(col_checker._bodies_information), 1)
        self.assertEqual(len(col_checker._external_collision_pairs), 0)

        # Re-add the second robot and verify.
        col_checker.add_body_id(robot_b.urdf)
        # Body will only be added once
        col_checker.add_body_id(robot_b.urdf)
        # Negative body ID should not be added
        col_checker.add_body_id(-1)
        # Visual box should not be added
        visual_box = create_visual_box([0, 0, 0])
        col_checker.add_body_id(visual_box)

        self.assertEqual(len(col_checker._bodies_information), 2)
        self.assertEqual(len(col_checker._external_collision_pairs), 1)

    def test_internal_collision_detection(self):
        """
        Verifies internal collision detection by:
          - Disabling internal collision checking and expecting a
            collision-free state.
          - Enabling collision checking and expecting collision detection.
          - Setting the safe state and then clearing it.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=False)
        self.assertTrue(col_checker.is_collision_free())

        col_checker.enable_internal_collision = True
        self.assertFalse(col_checker.is_collision_free())

        col_checker.set_safe_state()
        self.assertTrue(col_checker.is_collision_free())

        col_checker.clear_ignored_internal_collision()
        self.assertFalse(col_checker.is_collision_free())

    def test_ignore_external_collision(self):
        """
        Verifies that external collisions can be ignored either entirely
        for a body pair or for specific link pairs.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        robot_b = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        body_ids = [robot_a.urdf, robot_b.urdf]
        col_checker = pi.CollisionChecker(body_ids=body_ids)

        # Ignore all external collisions between the two robots.
        col_checker.ignore_external_body_collision((robot_a.urdf,
                                                    robot_b.urdf))
        # Adding same pair should not change the ignored pairs.
        col_checker.ignore_external_body_collision((robot_a.urdf,
                                                    robot_b.urdf))
        col_checker.ignore_external_link_collision((robot_a.urdf, robot_b.urdf),
                                                   [(-1, -1)])
        for bp, link_pairs in col_checker._external_collision_pairs:
            if set(bp) == set((robot_a.urdf, robot_b.urdf)):
                self.assertEqual(link_pairs, [])

        col_checker.clear_ignored_external_collision()

        # Ignore specific link pairs.
        pair_info = col_checker._external_collision_pairs[0]
        ignored_links = [pair_info[1][0]]
        col_checker.ignore_external_link_collision(pair_info[0],
                                                   ignored_links)
        # Adding same pair should not change the ignored pairs.
        col_checker.ignore_external_link_collision(pair_info[0],
                                                   ignored_links)
        for bp, link_pairs in col_checker._external_collision_pairs:
            if set(bp) == set(pair_info[0]):
                for lp in ignored_links:
                    self.assertNotIn(lp, link_pairs)

    def test_get_internal_collisions(self):
        """
        Verifies that internal collisions are reported as a list of colliding
        link pairs for a given body.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=True)
        collisions = col_checker.get_internal_collisions()
        self.assertIsInstance(collisions, list)
        self.assertFalse(len(collisions) == 0)

    def test_get_external_collisions_and_distance(self):
        """
        Verifies that external collisions and distances between bodies are
        computed correctly, using both instance and static methods.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        # Shift the second robot to ensure contact.
        shifted_pos = self.start_pos + np.array([0.1, 0, 0])
        robot_b = pi.RobotBase(self.urdf_robot, shifted_pos,
                               self.start_orient)
        body_ids = [robot_a.urdf, robot_b.urdf]
        col_checker = pi.CollisionChecker(body_ids=body_ids,
                                          max_distance_external=0.1)

        external_collisions = col_checker.get_external_collisions()
        self.assertIsInstance(external_collisions, list)
        self.assertFalse(len(external_collisions) == 0)

        initial_threshold = 0.1
        distance = col_checker.get_body_distance(robot_a.urdf, robot_b.urdf,
                                                 initial_threshold)
        self.assertLessEqual(distance, initial_threshold)

        static_collisions = pi.CollisionChecker.get_bodies_external_collisions(
            robot_a.urdf, robot_b.urdf, max_distance=0.1)
        self.assertIsInstance(static_collisions, list)
        self.assertFalse(len(static_collisions) == 0)

    def test_ignore_internal_link_collision(self):
        """
        Verifies that specified internal collisions between selected link
        pairs can be ignored.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=True)
        links = col_checker._bodies_information[0]['collision_links']
        ignore_pair = (links[0], links[1])
        col_checker.ignore_internal_link_collision(
            robot_a.urdf, (ignore_pair[0], ignore_pair[1]))
        # Adding same pair should not change the ignored pairs.
        col_checker.ignore_internal_link_collision(
            robot_a.urdf, (ignore_pair[0], ignore_pair[1]))
        collisions_after = col_checker.get_internal_collisions()
        for body_id, colliding_pairs in collisions_after:
            self.assertNotIn(ignore_pair, colliding_pairs)

    def test_controlled_distance_and_collision(self):
        """
        Verifies distance computation by creating two boxes with a known gap,
        then resetting them to the same position to simulate collision.
        Both get_body_distance and get_external_distance are validated.
        """
        # Create two boxes with an approximate gap of 0.1.
        box1_id = create_collision_box([0, 0, 0])
        box2_id = create_collision_box([1.1, 0, 0])
        col_checker = pi.CollisionChecker(
            body_ids=[box1_id, box2_id], max_distance_external=0.11)

        # Validate computed gap using get_body_distance.
        distance = col_checker.get_body_distance(box1_id, box2_id, 0.11)
        self.assertAlmostEqual(distance, 0.1, delta=1e-2)

        # Distance at least treshold
        distance = col_checker.get_body_distance(box1_id, box2_id, 0.0)
        self.assertEqual(distance, 0.0)

        # Validate minimum external distance using get_external_distance.
        min_distance = col_checker.get_external_distance(0.11)
        self.assertAlmostEqual(min_distance, 0.1, delta=1e-2)

        # Reset the boxes to the same position to simulate collision.
        reset_pos = [0, 0, 0]
        p.resetBasePositionAndOrientation(box1_id, reset_pos, [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(box2_id, reset_pos, [0, 0, 0, 1])

        # Recompute distance after resetting positions.
        distance_after_reset = col_checker.get_body_distance(box1_id, box2_id,
                                                             0.0)
        self.assertFalse(col_checker.is_collision_free())
        self.assertFalse(col_checker.is_external_collision_free())
        self.assertAlmostEqual(distance_after_reset, -1.0, delta=1e-8)
        col_checker.enable_external_collision = False
        self.assertTrue(col_checker.is_external_collision_free())
        visual_box = create_visual_box([0, 0, 0])
        distance = col_checker.get_body_distance(box1_id, visual_box, 1.0)
        self.assertEqual(distance, 1.0)


if __name__ == '__main__':
    unittest.main()
