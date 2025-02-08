import pybullet as p
from itertools import combinations, product

BASE_LINK = -1
MAX_DISTANCE = 0.0


class CollisionChecker:
    """
    Manages collision geometry for bodies in a PyBullet simulation and
    categorizes them based on joint mobility.

    For each body with collision geometry, an entry is added to
    `bodies_information` (a list of dicts) containing:
      - 'urdf_id': The unique identifier of the body.
      - 'body_typ': 'robot' if the body has movable joints,
                    'static_body' otherwise.
      - 'collision_links': A sorted list of link indices that have collision
                           geometry, including the base.
      - 'collision_pairs': Pairs of links (tuples) representing internal
                           collision checks.
    """

    def __init__(self, ignored_urdf_ids: list = [],
                 ignored_internal_collisions: list = [],
                 ignored_external_collisions: list = []):
        """
        Initializes the CollisionChecker by collecting collision data from all
        bodies in the simulation (except those specified in ignored_urdf_ids),
        and building lists for both internal and external collision pairs.

        Args:
            ignored_urdf_ids (list): List of urdf_ids to ignore (skip).
            ignored_internal_collisions (list): List of internal collision pairs
                                                to ignore.
            ignored_external_collisions (list): List of external collision pair
                                                specifications to ignore.
        """
        self.ignored_urdf_ids = ignored_urdf_ids
        self.bodies_information = []
        self.build_bodies_information(ignored_urdf_ids)
        self.ignored_internal_collisions = ignored_internal_collisions
        self.ignored_external_collisions = ignored_external_collisions
        self.internal_collision_pairs = []
        self.external_collision_pairs = []
        self.update_internal_collision_pairs()
        self.update_external_collision_pairs()

    def build_bodies_information(self, ignored_urdf_ids: list):
        """
        Iterates through all bodies in the simulation and collects collision
        geometry information. Only bodies with collision geometry are included,
        and bodies with urdf_ids in ignored_urdf_ids are skipped.

        Determines whether a body is a 'robot' (if it has at least one movable
        joint) or a 'static_body'. Also precomputes internal collision pairs for
        each body.
        """
        num_bodies = p.getNumBodies()
        for urdf_id in range(num_bodies):
            if urdf_id in ignored_urdf_ids:
                continue
            links = CollisionChecker.get_collision_links_for_body(urdf_id)
            # Skip bodies that lack any collision geometry.
            if not links:
                continue
            body_typ = ('robot'
                        if CollisionChecker.has_moving_joint(urdf_id)
                        else 'static_body')
            self.bodies_information.append({
                'urdf_id': urdf_id,
                'body_typ': body_typ,
                'collision_links': links,
                'collision_pairs':
                    CollisionChecker.build_internal_collision_pairs(links)
            })

    def update_internal_collision_pairs(self):
        """
        Updates the list of internal collision pairs for each robot body.
        For each robot, any collision pair specified in the ignored list is
        removed. The result is stored in
        `self.internal_collision_pairs` as a list of tuples:
            (urdf_id, [list of link pairs])
        """
        self.internal_collision_pairs = []

        for body in self.bodies_information:
            if body['body_typ'] == 'robot':
                # Work on a copy to avoid modifying the original list.
                valid_pairs = body['collision_pairs'].copy()
                # Remove any collision pairs that are marked to be ignored.
                for ignored in self.ignored_internal_collisions:
                    if ignored[0] == body['urdf_id']:
                        for pair in ignored[1]:
                            if pair in valid_pairs:
                                valid_pairs.remove(pair)
                self.internal_collision_pairs.append(
                    (body['urdf_id'], valid_pairs)
                )

    def update_external_collision_pairs(self):
        """
        Updates the list of external collision pairs between bodies.
        Each entry in `self.external_collision_pairs` is a tuple:
            ((bodyA, bodyB), [list of link pairs])

        For each pair of bodies, if the pair is specified in the ignored list,
        the corresponding link pairs are removed from the full Cartesian
        product of their collision links.
        """
        # Retrieve all body IDs from the collected collision information.
        body_ids = [body['urdf_id'] for body in self.bodies_information]
        potential_body_pairs = list(combinations(body_ids, 2))
        self.external_collision_pairs = []

        # Process each potential body pair.
        for bodyA, bodyB in potential_body_pairs:
            # If the body pair is in the ignored list, process the entry.
            ignored_entry = None
            for entry in self.ignored_external_collisions:
                # Each ignored entry is a tuple:
                # ((bodyA, bodyB), [ignored link pairs]).
                if entry[0] == (bodyA, bodyB):
                    ignored_entry = entry
                    break

            # Retrieve the collision links for both bodies.
            linksA = CollisionChecker.get_collision_links_for_body(bodyA)
            linksB = CollisionChecker.get_collision_links_for_body(bodyB)
            # Generate all possible link pairs (Cartesian product).
            link_pairs = CollisionChecker.build_external_collision_pairs(
                linksA, linksB
            )

            if ignored_entry is not None:
                for ignore_pair in ignored_entry[1]:
                    if ignore_pair in link_pairs:
                        link_pairs.remove(ignore_pair)
            self.external_collision_pairs.append(
                ((bodyA, bodyB), link_pairs)
            )

    def check_collision(self):
        """
        Checks for collisions across all robot bodies and between different
        bodies.

        Returns:
            bool: True if no collisions are detected, False if at least one
                  collision is found.
        """
        return (self.check_internal_collisions() and
                self.check_external_collisions())

    def check_internal_collisions(self):
        """
        Checks for collisions between internal link pairs of a robot body.

        Returns:
            bool: True if no internal collisions are detected, False otherwise.
        """
        for urdf_id, pairs in self.internal_collision_pairs:
            for linkA, linkB in pairs:
                if CollisionChecker.simple_collision(
                        urdf_id, urdf_id, linkA, linkB):
                    return False
        return True

    def check_external_collisions(self):
        """
        Checks for collisions between external link pairs from different bodies.

        Returns:
            bool: True if no external collisions are detected, False otherwise.
        """
        for (bodyA, bodyB), link_pairs in self.external_collision_pairs:
            for linkA, linkB in link_pairs:
                if CollisionChecker.simple_collision(bodyA, bodyB,
                                                     linkA, linkB):
                    return False
        return True

    def get_internal_collisions(self):
        """
        Determines the internal collision pairs that are actually colliding.

        Returns:
            list: A list of tuples, each containing a urdf_id and a list of
                  colliding link pairs.
        """
        internal_collisions = []
        for entry in self.internal_collision_pairs:
            link_pair_collisions = []
            for linkA, linkB in entry[1]:
                if CollisionChecker.simple_collision(
                        entry[0], entry[0], linkA, linkB):
                    link_pair_collisions.append((linkA, linkB))
            if len(link_pair_collisions) > 0:
                internal_collisions.append((entry[0], link_pair_collisions))
        return internal_collisions

    def get_global_external_collisions(self):
        """
        Checks all external collision pairs and returns the pairs that are
        colliding.

        Returns:
            list: A list of tuples where each tuple contains a body pair and
                  the list of colliding link pairs.
        """
        global_external_collisions = []

        for (bodyA, bodyB), link_pairs in self.external_collision_pairs:
            colliding_links = []
            for linkA, linkB in link_pairs:
                if CollisionChecker.simple_collision(bodyA, bodyB,
                                                     linkA, linkB):
                    colliding_links.append((linkA, linkB))
            if colliding_links:
                global_external_collisions.append(
                    ((bodyA, bodyB), colliding_links)
                )

        return global_external_collisions

    def set_safe_state(self):
        """
        Sets the current collisions as the safe state. This method retrieves
        the current internal and external collisions from the bodies in
        bodies_information and stores them as ignored collisions. It then
        updates the internal and external collision pair lists.
        """
        self.ignored_internal_collisions = self.get_internal_collisions()
        self.ignored_external_collisions = self.get_global_external_collisions()
        self.update_internal_collision_pairs()
        self.update_external_collision_pairs()

    @staticmethod
    def get_local_external_collisions(bodyA: int, bodyB: int):
        """
        Determines the external collision pairs that are colliding between two
        specific bodies.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.

        Returns:
            list: A list of colliding link pairs (tuples).
        """
        linksA = CollisionChecker.get_collision_links_for_body(bodyA)
        linksB = CollisionChecker.get_collision_links_for_body(bodyB)
        external_collision_pairs = (
            CollisionChecker.build_external_collision_pairs(linksA, linksB)
        )
        external_collisions = []
        for linkA, linkB in external_collision_pairs:
            if CollisionChecker.simple_collision(bodyA, bodyB, linkA, linkB):
                external_collisions.append((linkA, linkB))
        return external_collisions

    @staticmethod
    def has_moving_joint(urdf_id: int):
        """
        Determines if the specified body has at least one movable joint.

        Args:
            urdf_id (int): The unique identifier of the body.

        Returns:
            bool: True if any joint is movable; False otherwise.
        """
        num_joints = p.getNumJoints(urdf_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(urdf_id, j)
            # The joint type is at index 2 in the returned tuple.
            if joint_info[2] != p.JOINT_FIXED:
                return True
        return False

    @staticmethod
    def get_collision_links_for_body(urdf_id: int):
        """
        Retrieves a sorted list of link indices that have collision geometry
        for the given body. This includes the base link.

        Args:
            urdf_id (int): The unique identifier of the body.

        Returns:
            list: Sorted list of link indices with collision geometry.
        """
        links_with_collision = []
        # Check the base link.
        if p.getCollisionShapeData(urdf_id, BASE_LINK):
            links_with_collision.append(BASE_LINK)
        # Check all non-base links.
        num_joints = p.getNumJoints(urdf_id)
        for link in range(num_joints):
            if p.getCollisionShapeData(urdf_id, link):
                links_with_collision.append(link)
        return sorted(links_with_collision)

    @staticmethod
    def build_internal_collision_pairs(links: list):
        """
        Generates all unique pairs of links from the provided list for internal
        collision checking.

        Args:
            links (list): A list of link indices.

        Returns:
            list: A list of tuples, each containing a unique pair of link
                  indices.
        """
        return list(combinations(links, 2))

    @staticmethod
    def build_external_collision_pairs(links1: list, links2: list):
        """
        Generates all pairs combining one link from each of the two lists for
        external collision checking.

        Args:
            links1 (list): List of link indices from the first body.
            links2 (list): List of link indices from the second body.

        Returns:
            list: A list of tuples representing all combinations of link pairs.
        """
        return list(product(links1, links2))

    @staticmethod
    def simple_collision(bodyA: int, bodyB: int, linkA: int, linkB: int):
        """
        Determines if a collision is occurring between a specific pair of links
        from two bodies.

        Args:
            bodyA (int): ID of the first body.
            bodyB (int): ID of the second body.
            linkA (int): Index of the link from the first body.
            linkB (int): Index of the link from the second body.

        Returns:
            bool: True if a collision is detected; False otherwise.
        """
        contacts = p.getClosestPoints(
            bodyA, bodyB, MAX_DISTANCE, linkIndexA=linkA,
            linkIndexB=linkB
        )
        return len(contacts) > 0
