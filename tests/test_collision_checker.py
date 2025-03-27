import os
import unittest
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi


def setup_environment():
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
    box_shape = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[0.5, 0.5, 0.5])
    box_visual = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[0.5, 0.5, 0.5])
    box_id = p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=box_shape,
                               baseVisualShapeIndex=box_visual,
                               basePosition=start_pos)
    return box_id


class TestCollisionChecker(unittest.TestCase):

    def setUp(self):
        (self.urdf_robot, self.start_pos,
         self.start_orient) = setup_environment()

    def tearDown(self):
        p.disconnect()

    def test_collision_pairs(self):
        """
        Tests methods of the CollisionChecker class using two overlapping
        robots. Body IDs are obtained from the robot.urdf attribute.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        robot_b = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        body_ids = [robot_a.urdf, robot_b.urdf]
        col_checker = pi.CollisionChecker(body_ids=body_ids)

        # Verify that bodies_information includes both robots.
        self.assertEqual(len(col_checker._bodies_information),
                         len(body_ids))
        for body in col_checker._bodies_information:
            self.assertEqual(body['body_type'], 'robot')

        # Verify internal collision pairs exist for each robot.
        expected_internal = len([b for b in col_checker._bodies_information
                                 if b['body_type'] == 'robot'])
        self.assertEqual(len(col_checker._internal_collision_pairs),
                         expected_internal)
        # External collision pairs are computed for one body pair.
        self.assertEqual(len(col_checker._external_collision_pairs), 1)

        # Make the second robot static.
        col_checker.make_robot_static(robot_b.urdf)
        entry_robot_b = [body for body in col_checker._bodies_information
                         if body['body_id'] == robot_b.urdf][0]
        self.assertEqual(entry_robot_b['body_type'], 'static_body')
        expected_internal = len([b for b in col_checker._bodies_information
                                 if b['body_type'] == 'robot'])
        self.assertEqual(len(col_checker._internal_collision_pairs),
                         expected_internal)

        # Remove the second robot.
        col_checker.remove_body_id(robot_b.urdf)
        self.assertEqual(len(col_checker._bodies_information), 1)
        self.assertEqual(len(col_checker._external_collision_pairs), 0)

        # Re-add the second robot.
        col_checker.add_body_id(robot_b.urdf)
        self.assertEqual(len(col_checker._bodies_information), 2)
        self.assertEqual(len(col_checker._external_collision_pairs), 1)

    def test_internal_collision_detection(self):
        """
        Tests internal collision detection by toggling collision control.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=False)
        # With internal collision checking disabled, a collision-free
        # state is reported.
        self.assertTrue(col_checker.is_collision_free())

        # Activate internal collision checking.
        col_checker.enable_internal_collision = True
        self.assertFalse(col_checker.is_collision_free())

        # Set the current state as safe.
        col_checker.set_safe_state()
        self.assertTrue(col_checker.is_collision_free())

        # Clearing ignored internal collisions should restore collision
        # detection.
        col_checker.clear_ignored_internal_collision()
        self.assertFalse(col_checker.is_collision_free())

    def test_ignore_external_collision(self):
        """
        Tests ignored external collisions for an entire body pair and
        specific link pairs.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        robot_b = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        body_ids = [robot_a.urdf, robot_b.urdf]
        col_checker = pi.CollisionChecker(body_ids=body_ids)

        # Ignore all external collisions between robot_a and robot_b.
        col_checker.ignore_external_body_collision((robot_a.urdf,
                                                    robot_b.urdf))
        for bp, link_pairs in col_checker._external_collision_pairs:
            if set(bp) == set((robot_a.urdf, robot_b.urdf)):
                self.assertEqual(link_pairs, [])

        # Clear ignored external collisions.
        col_checker.clear_ignored_external_collision()

        # If external collision pairs exist, ignore specific link pairs.
        pair_info = col_checker._external_collision_pairs[0]
        if pair_info[1]:
            ignored_links = [pair_info[1][0]]
            col_checker.ignore_external_link_collision(pair_info[0],
                                                       ignored_links)
            # Verify that the ignored link pair is removed.
            for bp, link_pairs in col_checker._external_collision_pairs:
                if set(bp) == set(pair_info[0]):
                    for lp in ignored_links:
                        self.assertNotIn(lp, link_pairs)

    def test_get_internal_collisions(self):
        """
        Tests the get_internal_collisions method.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=True)
        collisions = col_checker.get_internal_collisions()
        # Assuming that a default robot configuration yields internal
        # collisions.
        self.assertIsInstance(collisions, list)
        self.assertFalse(len(collisions) == 0)

    def test_get_external_collisions_and_distance(self):
        """
        Tests the get_external_collisions method, get_distance, and the static
        get_bodies_external_collisions method.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        # Shift second robot slightly to ensure contact.
        shifted_pos = self.start_pos + np.array([0.1, 0, 0])
        robot_b = pi.RobotBase(self.urdf_robot, shifted_pos,
                               self.start_orient)
        body_ids = [robot_a.urdf, robot_b.urdf]
        col_checker = pi.CollisionChecker(body_ids=body_ids,
                                          max_distance_external=0.1)

        external_collisions = col_checker.get_external_collisions()
        self.assertIsInstance(external_collisions, list)
        self.assertFalse(len(external_collisions) == 0)

        # Test get_distance between the two bodies.
        initial_threshold = 0.1
        distance = col_checker.get_body_distance(robot_a.urdf, robot_b.urdf,
                                                 initial_threshold)
        self.assertLessEqual(distance, initial_threshold)

        # Use the static method to fetch external collisions.
        static_collisions = pi.CollisionChecker.get_bodies_external_collisions(
            robot_a.urdf, robot_b.urdf, max_distance=0.1)
        self.assertIsInstance(static_collisions, list)
        self.assertFalse(len(static_collisions) == 0)

    def test_ignore_internal_link_collision(self):
        """
        Tests the ignore_internal_link_collision method.
        """
        robot_a = pi.RobotBase(self.urdf_robot, self.start_pos,
                               self.start_orient)
        col_checker = pi.CollisionChecker(body_ids=[robot_a.urdf],
                                          enable_internal_collision=True)
        # Use the first body's collision links.
        links = col_checker._bodies_information[0]['collision_links']
        if len(links) >= 2:
            ignore_pair = (links[0], links[1])
            col_checker.ignore_internal_link_collision(
                robot_a.urdf, (ignore_pair[0], ignore_pair[1]))
            collisions_after = col_checker.get_internal_collisions()
            # The ignored pair should no longer appear.
            for body_id, colliding_pairs in collisions_after:
                self.assertNotIn(ignore_pair, colliding_pairs)

    def test_controlled_distance_and_collision(self):
        """
        Creates two boxes at known positions, verifies that the computed
        distance is approximately the expected gap, then resets the boxes
        to induce collision and verifies that the computed distance
        is near zero. The test also verifies get_external_distance.
        """
        # Create two boxes with a gap.
        box1_id = create_collision_box([0, 0, 0])
        box2_id = create_collision_box([1.1, 0, 0])
        col_checker = pi.CollisionChecker(
            body_ids=[box1_id, box2_id], max_distance_external=0.11)

        # Compute distance using get_body_distance.
        distance = col_checker.get_body_distance(box1_id, box2_id, 0.11)
        self.assertAlmostEqual(distance, 0.1, delta=1e-2)

        # Verify minimum external distance using get_external_distance.
        min_distance = col_checker.get_external_distance(0.11)
        self.assertAlmostEqual(min_distance, 0.1, delta=1e-2)

        # Reset boxes to the same position to induce collision.
        reset_pos = [0, 0, 0]
        p.resetBasePositionAndOrientation(box1_id, reset_pos, [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(box2_id, reset_pos, [0, 0, 0, 1])

        # Recompute distance after resetting positions.
        distance_after_reset = col_checker.get_body_distance(
            box1_id, box2_id, 0.0)
        self.assertAlmostEqual(distance_after_reset, -1.0, delta=1e-8)


if __name__ == '__main__':
    unittest.main()
