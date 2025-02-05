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

    def __init__(self):
        """
        Initializes the CollisionChecker and builds the list of
        bodies with collision information.
        """
        self.bodies_information = []
        self.external_collisions = []
        self.build_collision_objects()
        self.uppdate_collision_settings()
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

    def uppdate_collision_settings(self):
        """
        Checks for internal collisions for each body based on its
        collision pairs.
        """
        for body in self.bodies_information:
            body['active_collisions'] = self.collision_pairs(
                body['urdf_id'],
                body['urdf_id'],
                body['collision_pairs']
            )

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
