import pybullet as p
from itertools import combinations, product

BASE_LINK = -1
MAX_DISTANCE_INTERNAL = 0.0


class CollisionChecker:
    """
    Manages collision geometry for bodies in a PyBullet sim and
    categorizes them based on joint mobility.

    For each body with collision geometry, an entry is added to
    `bodies_information` (a list of dicts) containing:
      - 'urdf_id': Unique identifier of the body.
      - 'urdf_name': Name of the URDF (decoded from bytes).
      - 'body_typ': 'robot' if the body has movable joints, otherwise
        'static_body'.
      - 'collision_links': Sorted list of link indices with collision
        geometry, including the base.
      - 'collision_pairs': Link pairs (tuples) for internal collision
        checks.
    """

    def __init__(self, urdf_ids: list = None,
                 ignored_internal_collisions: list = [],
                 ignored_external_collisions: list = [],
                 max_distance_external: float = 0.0,
                 enable_internal_collision: bool = True,
                 enable_external_collision: bool = True):
        """
        Initializes CollisionChecker by collecting collision data from
        selected bodies in the simulation (if urdf_ids is provided, only
        those bodies are processed) and by building lists for both internal
        and external collision pairs.

        Args:
            urdf_ids (list): List of URDF IDs to consider. If None, all
                bodies are processed.
            ignored_internal_collisions (list): List of internal collision
                pairs to ignore. Each element is a tuple
                (urdf_id, [list of link pairs]).
            ignored_external_collisions (list): List of external collision
                specs to ignore. Each element is a tuple
                ((bodyA, bodyB), [ignored link pairs]).
            max_distance_external (float): Maximum distance threshold for
                external collision detection.
            enable_internal_collision (bool): Flag to enable internal
                collision checking.
            enable_external_collision (bool): Flag to enable external
                collision checking.
        """
        self.max_distance_external = max_distance_external
        self.urdf_ids = urdf_ids  # Only these bodies are considered (if given)
        self.bodies_information = []
        self.build_bodies_information(urdf_ids)
        self.ignored_internal_collisions = ignored_internal_collisions
        self.ignored_external_collisions = ignored_external_collisions
        self.internal_collision_pairs = []
        self.external_collision_pairs = []
        self.enable_internal_collision = enable_internal_collision
        self.enable_external_collision = enable_external_collision
        self.update_internal_collision_pairs()
        self.update_external_collision_pairs()

    def build_bodies_information(self, urdf_ids: list):
        """
        Iterates through all bodies in the sim and collects collision
        geometry information. If a list of URDF IDs is provided, only those
        bodies are processed.

        For each body, determines whether it is a 'robot' (if it has at least
        one movable joint) or a 'static_body'. It also computes internal
        collision pairs.

        Additionally, the URDF name is decoded from bytes and stored as
        'urdf_name'.
        """
        num_bodies = p.getNumBodies()
        for urdf_id in range(num_bodies):
            if urdf_ids is not None and urdf_id not in urdf_ids:
                continue

            # Retrieve body info; p.getBodyInfo returns a tuple of two byte
            # strings: (bodyName, additionalInfo). Decode additionalInfo.
            body_info = p.getBodyInfo(urdf_id)
            urdf_name = body_info[1].decode('utf-8')

            # Get collision link indices for this body.
            links = CollisionChecker.get_collision_links_for_body(urdf_id)
            # Skip bodies with no collision geometry.
            if not links:
                continue

            # Determine if the body has any movable joints.
            body_typ = (
                'robot'
                if CollisionChecker.has_moving_joint(urdf_id)
                else 'static_body'
            )
            self.bodies_information.append({
                'urdf_id': urdf_id,
                'urdf_name': urdf_name,
                'body_typ': body_typ,
                'collision_links': links,
                'collision_pairs': CollisionChecker.
                    build_internal_collision_pairs(links)
            })

    def update_internal_collision_pairs(self, ignore_collision: bool = True):
        """
        Updates the list of internal collision pairs for each robot body.
        For each robot, collision pairs specified in the ignored list are
        removed (if ignore_collision is True). The resulting pairs are stored
        in self.internal_collision_pairs as a list of tuples:
        (urdf_id, [list of valid link pairs]).

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.
        """
        self.internal_collision_pairs = []

        for body in self.bodies_information:
            if body['body_typ'] == 'robot':
                # Work on a copy to avoid modifying the original list.
                valid_pairs = body['collision_pairs'].copy()
                # Remove collision pairs that are to be ignored.
                if ignore_collision:
                    for ignored in self.ignored_internal_collisions:
                        if ignored[0] == body['urdf_id']:
                            for pair in ignored[1]:
                                if pair in valid_pairs:
                                    valid_pairs.remove(pair)
                self.internal_collision_pairs.append(
                    (body['urdf_id'], valid_pairs)
                )

    def update_external_collision_pairs(self, ignore_collision: bool = True):
        """
        Updates the list of external collision pairs between bodies.
        Each entry in self.external_collision_pairs is a tuple:
            ((bodyA, bodyB), [list of link pairs]).

        For each body pair, if the pair is in the ignored list, the
        corresponding link pairs are removed from the complete Cartesian
        product of their collision links.

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.
        """
        body_ids = [body['urdf_id'] for body in self.bodies_information]
        potential_body_pairs = list(combinations(body_ids, 2))
        self.external_collision_pairs = []

        for bodyA, bodyB in potential_body_pairs:
            ignored_entry = None
            if ignore_collision:
                # Look for an ignored collision entry for this pair.
                for entry in self.ignored_external_collisions:
                    if entry[0] == (bodyA, bodyB):
                        ignored_entry = entry
                        break

            linksA = CollisionChecker.get_collision_links_for_body(bodyA)
            linksB = CollisionChecker.get_collision_links_for_body(bodyB)
            link_pairs = CollisionChecker.build_external_collision_pairs(
                linksA, linksB
            )

            if ignored_entry is not None:
                for ignore_pair in ignored_entry[1]:
                    if ignore_pair in link_pairs:
                        link_pairs.remove(ignore_pair)
            self.external_collision_pairs.append(((bodyA, bodyB), link_pairs))

    def check_collision(self) -> bool:
        """
        Checks for collisions within individual robot bodies and between
        different bodies.

        Returns:
            bool: True if no collisions are detected; False otherwise.
        """
        if self.enable_internal_collision:
            if not self.check_internal_collisions():
                return False
        if self.enable_external_collision:
            if not self.check_external_collisions():
                return False
        return True

    def check_internal_collisions(self) -> bool:
        """
        Checks for collisions between internal link pairs within each robot.

        Returns:
            bool: True if no internal collisions are detected; False if any are.
        """
        if self.enable_internal_collision:
            for urdf_id, pairs in self.internal_collision_pairs:
                for linkA, linkB in pairs:
                    if CollisionChecker.simple_collision(
                        urdf_id, urdf_id, MAX_DISTANCE_INTERNAL,
                        linkA, linkB
                    ):
                        return False
        return True

    def check_external_collisions(self) -> bool:
        """
        Checks for collisions between link pairs from different bodies.

        Returns:
            bool: True if no external collisions are detected; False otherwise.
        """
        if self.enable_external_collision:
            for (bodyA, bodyB), link_pairs in self.external_collision_pairs:
                for linkA, linkB in link_pairs:
                    if CollisionChecker.simple_collision(
                        bodyA, bodyB, self.max_distance_external,
                        linkA, linkB
                    ):
                        return False
        return True

    def get_internal_collisions(self) -> list:
        """
        Identifies and returns internal collision pairs that are colliding
        within each robot.

        Returns:
            list: A list of tuples, each containing a urdf_id and a list of
            colliding link pairs.
        """
        internal_collisions = []
        for entry in self.internal_collision_pairs:
            link_pair_collisions = []
            for linkA, linkB in entry[1]:
                if CollisionChecker.simple_collision(
                    entry[0], entry[0], MAX_DISTANCE_INTERNAL,
                    linkA, linkB
                ):
                    link_pair_collisions.append((linkA, linkB))
            if link_pair_collisions:
                internal_collisions.append(
                    (entry[0], link_pair_collisions)
                )
        return internal_collisions

    def get_global_external_collisions(self) -> list:
        """
        Checks all external collision pairs between bodies and returns those
        that are colliding.

        Returns:
            list: A list of tuples where each tuple contains a body pair and a
            list of colliding link pairs.
        """
        global_external_collisions = []
        for (bodyA, bodyB), link_pairs in self.external_collision_pairs:
            colliding_links = []
            for linkA, linkB in link_pairs:
                if CollisionChecker.simple_collision(
                    bodyA, bodyB, self.max_distance_external,
                    linkA, linkB
                ):
                    colliding_links.append((linkA, linkB))
            if colliding_links:
                global_external_collisions.append(
                    ((bodyA, bodyB), colliding_links)
                )
        return global_external_collisions

    def set_safe_state(self):
        """
        Sets the current collision configuration as the safe state.

        This method updates the collision pair lists without filtering,
        records the current collisions as ignored, and then updates the lists
        to exclude these safe collisions.
        """
        self.update_internal_collision_pairs(ignore_collision=False)
        self.update_external_collision_pairs(ignore_collision=False)

        self.ignored_internal_collisions = self.get_internal_collisions()
        self.ignored_external_collisions = self.get_global_external_collisions()
        self.update_internal_collision_pairs()
        self.update_external_collision_pairs()

    def get_global_distance(self, distance: float) -> float:
        """
        Determines the smallest distance across all external collision pairs.

        Args:
            distance (float): An initial maximum distance threshold.

        Returns:
            float: The smallest distance found among the external pairs.
        """
        min_distance = distance
        for (bodyA, bodyB), _ in self.external_collision_pairs:
            curr_distance = self.get_local_distance(bodyA, bodyB,
                                                    min_distance)
            if curr_distance < min_distance:
                min_distance = curr_distance
        return min_distance

    def get_local_distance(self, bodyA: int, bodyB: int,
                           distance: float) -> float:
        """
        Determines the smallest distance between any collision pair (i.e.
        link pair) for a given pair of bodies by inspecting external
        collision pairs.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.
            distance (float): The initial maximum distance threshold.

        Returns:
            float: The smallest distance found among the pairs; if no
            contact is detected, returns the provided threshold.
        """
        pair_key = tuple(sorted([bodyA, bodyB]))
        for (bodies, link_pairs) in self.external_collision_pairs:
            if bodies == pair_key:
                min_distance = distance
                for linkA, linkB in link_pairs:
                    curr_dist = CollisionChecker.get_distance(
                        bodyA, bodyB, distance, linkA, linkB
                    )
                    if curr_dist < min_distance:
                        min_distance = curr_dist
                return min_distance
        return distance

    @staticmethod
    def get_local_external_collisions(bodyA: int, bodyB: int,
                                      max_distance: float = 0.0) -> list:
        """
        Determines the external collision pairs that are colliding between
        two specific bodies.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.
            max_distance (float): Maximum distance threshold for collision.

        Returns:
            list: A list of colliding link pairs (tuples) between the bodies.
        """
        linksA = CollisionChecker.get_collision_links_for_body(bodyA)
        linksB = CollisionChecker.get_collision_links_for_body(bodyB)
        ext_pairs = CollisionChecker.build_external_collision_pairs(
            linksA, linksB
        )
        external_collisions = []
        for linkA, linkB in ext_pairs:
            if CollisionChecker.simple_collision(
                bodyA, bodyB, max_distance, linkA, linkB
            ):
                external_collisions.append((linkA, linkB))
        return external_collisions

    @staticmethod
    def has_moving_joint(urdf_id: int) -> bool:
        """
        Determines if the specified body has at least one movable joint.

        Args:
            urdf_id (int): Unique identifier of the body.

        Returns:
            bool: True if at least one joint is movable; False if all are fixed.
        """
        num_joints = p.getNumJoints(urdf_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(urdf_id, j)
            if joint_info[2] != p.JOINT_FIXED:
                return True
        return False

    @staticmethod
    def get_collision_links_for_body(urdf_id: int) -> list:
        """
        Retrieves a sorted list of link indices that have collision
        geometry for the given body. This includes the base link.

        Args:
            urdf_id (int): Unique identifier of the body.

        Returns:
            list: Sorted list of link indices with collision shapes.
        """
        links_with_collision = []
        # Check if the base link has collision geometry.
        if p.getCollisionShapeData(urdf_id, BASE_LINK):
            links_with_collision.append(BASE_LINK)
        num_joints = p.getNumJoints(urdf_id)
        # Check each joint/link for collision geometry.
        for link in range(num_joints):
            if p.getCollisionShapeData(urdf_id, link):
                links_with_collision.append(link)
        return sorted(links_with_collision)

    @staticmethod
    def build_internal_collision_pairs(links: list) -> list:
        """
        Generates all unique pairs of links from the provided list for
        internal collision checking.

        Args:
            links (list): List of link indices.

        Returns:
            list: List of tuples, each containing a unique pair of links.
        """
        return list(combinations(links, 2))

    @staticmethod
    def build_external_collision_pairs(links1: list,
                                       links2: list) -> list:
        """
        Generates all pairs combining one link from each of two lists for
        external collision checking.

        Args:
            links1 (list): List of link indices from the first body.
            links2 (list): List of link indices from the second body.

        Returns:
            list: List of tuples representing all combinations of link pairs.
        """
        return list(product(links1, links2))

    @staticmethod
    def simple_collision(bodyA: int, bodyB: int, max_distance: float,
                         linkA: int, linkB: int) -> bool:
        """
        Determines if a collision is occurring between a specific pair of
        links from two bodies using PyBullet's getClosestPoints.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.
            max_distance (float): Maximum distance threshold for collision.
            linkA (int): Index of the link from the first body.
            linkB (int): Index of the link from the second body.

        Returns:
            bool: True if a collision (or contact within max_distance) is
                detected; False otherwise.
        """
        contacts = p.getClosestPoints(
            bodyA, bodyB, max_distance,
            linkIndexA=linkA, linkIndexB=linkB
        )
        return len(contacts) > 0

    @staticmethod
    def get_distance(bodyA: int, bodyB: int, distance: float,
                     linkA: int, linkB: int) -> float:
        """
        Computes the closest distance between two specified links from
        two bodies. Uses PyBullet's getClosestPoints with the provided
        maximum distance threshold.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.
            distance (float): Maximum distance threshold.
            linkA (int): Index of the link on the first body.
            linkB (int): Index of the link on the second body.

        Returns:
            float: The smallest distance found between the two links if any
            contact is detected; otherwise, returns the provided threshold.
        """
        contacts = p.getClosestPoints(
            bodyA, bodyB, distance,
            linkIndexA=linkA, linkIndexB=linkB
        )
        if contacts:
            return min(contact[8] for contact in contacts)
        return distance
