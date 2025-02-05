import pybullet as p
from itertools import combinations, product

BASE_LINK = -1
MAX_DISTANCE = 0.0


class CollisionChecker:
    """
    Aggregates collision shape data for bodies in the simulation and
    categorizes them based on joint mobility.

    For each body with collision data, an entry is added to
    bodies_information (a list of dicts) with keys:
      - 'urdf_id': Unique body identifier.
      - 'body_typ': 'robot' if the body has any movable joint,
                    'static_body' otherwise.
      - 'collision_links': Sorted list of link indices with collision
                           shape data, including the base.
      - 'collision_pairs': Internal collision pairs built from the links.
    """

    def __init__(self, ignored_internal_collisions: list = [],
                 ignored_external_collisions: list = []):
        """
        Initializes the CollisionChecker and builds the list of
        bodies with collision information.
        """
        self.bodies_information = []
        self.external_collisions = []
        self.build_collision_objects()
        self.ignored_internal_collisions = ignored_internal_collisions
        self.ignored_external_collisions = ignored_external_collisions
        # Only her for test purposes
        self.ignore_current_internal_collisions(0)
        self.pairs_internal_collisions = []
        self.pairs_external_collisions = []
        self.uppdate_internal_collision_pairs()
        self.uppddate_external_collionn_pairs()
        self.check_internal_collisions()
        print("stop")

    def build_collision_objects(self):
        """
        Iterates over all bodies in the simulation and classifies each
        based on joint mobility and collision shape data.

        Bodies with no collision data are skipped.
        """
        num_bodies = p.getNumBodies()
        for body_id in range(num_bodies):
            links = CollisionChecker.get_collision_links_for_body(body_id)
            # Skip bodies with no collision shapes.
            if not links:
                continue
            if CollisionChecker.has_moving_joint(body_id):
                body_typ = 'robot'
            else:
                body_typ = 'static_body'
            self.bodies_information.append({
                'urdf_id': body_id,
                'body_typ': body_typ,
                'collision_links': links,
                'collision_pairs': self.build_internal_collision_pairs(links)
            })

    def uppdate_internal_collision_pairs(self):
        # For each body in bodies_information that is a robot,
        # adjust its collision pairs by removing any ignored pairs,
        # then add a record to check_internal_collisions.
        for body in self.bodies_information:
            if body['body_typ'] == 'robot':
                # Work on a copy to avoid modifying the original list.
                check_pairs = body['collision_pairs'].copy()
                # Look for a matching ignored record for this body.
                for ignored in self.ignored_internal_collisions:
                    if ignored['urdf_id'] == body['urdf_id']:
                        for pair in ignored['ignored_pairs']:
                            if pair in check_pairs:
                                check_pairs.remove(pair)
                # Append the result once per robot.
                self.pairs_internal_collisions.append({
                    'urdf_id': body['urdf_id'],
                    'check_pairs': check_pairs
                })

    def uppddate_external_collionn_pairs(self):
        """
        Builds a list of external collision pairs. Each entry is a list:
        [(bodyA, bodyB), [ (linkA, linkB), (linkA, linkB), ... ]].

        Bodies in a pair are skipped if they appear in ignored_external_collisions.
        """
        # Get all body ids from the collision bodies.
        body_ids = [body['urdf_id'] for body in self.bodies_information]
        potential_body_pairs = list(combinations(body_ids, 2))

        self.pairs_external_collisions = []
        for bodyA, bodyB in potential_body_pairs:
            # Skip this pair if it's in the ignored list (in either order).
            if ((bodyA, bodyB) in self.ignored_external_collisions or
                    (bodyB, bodyA) in self.ignored_external_collisions):
                continue

            # Get the collision links for each body.
            linksA = self.bodies_information[bodyA]['collision_links']
            linksB = self.bodies_information[bodyB]['collision_links']
            # Build all link pairs between these bodies.
            link_pairs = CollisionChecker.build_external_collision_pairs(linksA, linksB)
            self.pairs_external_collisions.append([(bodyA, bodyB), link_pairs])

    def check_collision(self):
        """
        Checks for collisions between all robot bodies and external objects.

        Returns:
            bool: True if no collisions are detected, False if at least one is found.
        """
        return self.check_internal_collisions() and self.check_external_collisions()

    def check_internal_collisions(self):
        """
        Checks internal collisions for all robot bodies.

        Returns:
            bool: True if no collisions are detected,
                False if at least one collision is found.
        """
        for body_entry in self.pairs_internal_collisions:
            body_id = body_entry['urdf_id']
            for linkA, linkB in body_entry['check_pairs']:
                if CollisionChecker.single_collision(body_id, body_id, linkA, linkB):
                    return False
        return True

    def check_external_collisions(self):
        """
        Checks external collisions between all body pairs.

        Returns:
            bool: True if no collisions are detected, False if at least one is found.
        """
        for pair_info in self.pairs_external_collisions:
            # Unpack the body pair and its link combinations.
            (bodyA, bodyB), link_pairs = pair_info
            for linkA, linkB in link_pairs:
                if CollisionChecker.single_collision(bodyA, bodyB, linkA, linkB):
                    return False
        return True

    def ignore_current_internal_collisions(self, urdf_id):
        """
        Ignores all current collisions for the given body.

        Args:
            urdf_id (int): Unique body identifier.
        """
        # Use the precomputed collision pairs instead of the collision links.
        ignore_pairs = self.collision_pairs(
            urdf_id, urdf_id,
            self.bodies_information[urdf_id]['collision_pairs']
        )
        # Create a dictionary entry rather than a list.
        ignored_entry = {
            'urdf_id': urdf_id,
            'ignored_pairs': ignore_pairs
        }
        self.ignored_internal_collisions.append(ignored_entry)
        self.uppdate_internal_collision_pairs()

    @staticmethod
    def has_moving_joint(body_id):
        """
        Checks if the given body has at least one movable joint.

        Args:
            body_id (int): Unique body identifier.

        Returns:
            bool: True if any joint is movable; else False.
        """
        num_joints = p.getNumJoints(body_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(body_id, j)
            # Joint type is at index 2.
            if joint_info[2] != p.JOINT_FIXED:
                return True
        return False

    @staticmethod
    def get_collision_links_for_body(body_id):
        """
        Retrieves all link indices that have collision shape data,
        including the base link.

        Args:
            body_id (int): Unique body identifier.

        Returns:
            list: Sorted list of link indices with collision shapes.
        """
        links_with_collision = []
        # Check base link.
        if p.getCollisionShapeData(body_id, BASE_LINK):
            links_with_collision.append(BASE_LINK)
        # Check non-base links.
        num_joints = p.getNumJoints(body_id)
        for link in range(num_joints):
            if p.getCollisionShapeData(body_id, link):
                links_with_collision.append(link)
        return sorted(links_with_collision)

    @staticmethod
    def collision_pairs(bodyA, bodyB, collision_pairs):
        """
        Checks for collisions between pairs of links.

        Args:
            bodyA (int): Identifier of the first body.
            bodyB (int): Identifier of the second body.
            collision_pairs (iterable): Pairs of link indices to check.

        Returns:
            list: List of tuples, each containing a colliding pair.
        """
        colls = []
        for linkA, linkB in collision_pairs:
            if CollisionChecker.single_collision(bodyA, bodyB,
                                                 linkA, linkB):
                # Create a tuple of the pair.
                colls.append((linkA, linkB))
        return colls

    @staticmethod
    def single_collision(bodyA, bodyB, linkA, linkB):
        """
        Determines if a specific pair of links is colliding.

        Args:
            bodyA (int): ID of the first body.
            bodyB (int): ID of the second body.
            linkA (int): Link index of bodyA.
            linkB (int): Link index of bodyB.

        Returns:
            bool: True if collision exists; else False.
        """
        contacts = p.getClosestPoints(bodyA, bodyB, MAX_DISTANCE,
                                      linkIndexA=linkA,
                                      linkIndexB=linkB)
        return len(contacts) > 0

    @staticmethod
    def build_internal_collision_pairs(links):
        """
        Returns all unique pairs from a list of links.

        Args:
            links (list): List of link indices.

        Returns:
            list: List of tuples with all unique link pairs.
        """
        return list(combinations(links, 2))

    @staticmethod
    def build_external_collision_pairs(links1, links2):
        """
        Returns all pairs combining one link from each list.

        Args:
            links1 (list): First list of link indices.
            links2 (list): Second list of link indices.

        Returns:
            list: List of tuples from the Cartesian product.
        """
        return list(product(links1, links2))
