import pybullet as p
from itertools import combinations, product

BASE_LINK = -1
MAX_DISTANCE_INTERNAL = 0.0


class CollisionChecker:
    """
    Manages collision geometry for bodies in a PyBullet sim and
    categorizes them based on joint mobility.

    Args:
        urdf_ids (list): List of URDF IDs to consider. If None, all
            bodies are processed.
        max_distance_external (float): Maximum distance threshold for
            external collision detection.
        enable_internal_collision (bool): Flag to enable internal
            collision checking.
        enable_external_collision (bool): Flag to enable external
            collision checking.
    """

    def __init__(self, urdf_ids: list = None,
                 max_distance_external: float = 0.0,
                 enable_internal_collision: bool = True,
                 enable_external_collision: bool = True):

        self.max_distance_external = max_distance_external
        self.urdf_ids = urdf_ids
        self.bodies_information = []
        # Private ignored collision lists.
        # __ignored_internal_collisions is a list of tuples:
        # (urdf_id, [ignored internal link pairs])
        self.__ignored_internal_collisions = []
        # __ignored_external_collisions is a list of tuples:
        # ((bodyA, bodyB), ignored link pairs or "ALL")
        self.__ignored_external_collisions = []
        # Private collision pair lists.
        self.__internal_collision_pairs = []
        self.__external_collision_pairs = []
        self.enable_internal_collision = enable_internal_collision
        self.enable_external_collision = enable_external_collision

        self.__build_bodies_information(urdf_ids)
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def __build_bodies_information(self, urdf_ids: list) -> None:
        """
        Iterates through all bodies in the sim and collects collision
        geometry information. If a list of URDF IDs is provided, only those
        bodies are processed.

        For each body, determines whether it is a 'robot' (if it has at least
        one movable joint) or a 'static_body'. It also computes internal
        collision pairs.

        Additionally, the URDF name is decoded from bytes and stored as
        'urdf_name'.

        Args:
            urdf_ids (list): List of URDF IDs to process.

        Returns:
            None
        """
        num_bodies = p.getNumBodies()
        for urdf_id in range(num_bodies):
            if urdf_ids is not None and urdf_id not in urdf_ids:
                continue
            body_info = p.getBodyInfo(urdf_id)
            urdf_name = body_info[1].decode('utf-8')
            links = CollisionChecker.__get_collision_links_for_body(urdf_id)
            if not links:
                continue
            body_type = ('robot'
                         if CollisionChecker.has_moving_joint(urdf_id)
                         else 'static_body')
            self.bodies_information.append({
                'urdf_id': urdf_id,
                'urdf_name': urdf_name,
                'body_type': body_type,
                'collision_links': links,
                'collision_pairs':
                    CollisionChecker.__build_internal_collision_pairs(links)
            })

    def __update_internal_collision_pairs(
            self, ignore_collision: bool = True) -> None:
        """
        Updates the list of internal collision pairs for each robot body.
        For each robot, collision pairs specified in the ignored list are
        removed (if ignore_collision is True). The resulting pairs are stored
        in __internal_collision_pairs as a list of tuples:
        (urdf_id, [list of valid link pairs]).

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.

        Returns:
            None
        """
        self.__internal_collision_pairs = []
        for body in self.bodies_information:
            if body['body_type'] == 'robot':
                valid_pairs = body['collision_pairs'].copy()
                if ignore_collision:
                    for ignored in self.__ignored_internal_collisions:
                        if ignored[0] == body['urdf_id']:
                            for pair in ignored[1]:
                                normalized_pair = tuple(sorted(pair))
                                if normalized_pair in valid_pairs:
                                    valid_pairs.remove(normalized_pair)
                self.__internal_collision_pairs.append(
                    (body['urdf_id'], valid_pairs)
                )

    def __update_external_collision_pairs(
            self, ignore_collision: bool = True) -> None:
        """
        Updates the list of external collision pairs between bodies.
        Each entry in __external_collision_pairs is a tuple:
        ((bodyA, bodyB), [list of link pairs]).

        For each body pair, if the pair is in the ignored list, the
        corresponding link pairs are removed from the complete Cartesian
        product of their collision links.

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.

        Returns:
            None
        """
        body_ids = [body['urdf_id'] for body in self.bodies_information]
        potential_body_pairs = list(combinations(body_ids, 2))
        self.__external_collision_pairs = []
        for bodyA, bodyB in potential_body_pairs:
            ignored_entry = None
            if ignore_collision:
                for entry in self.__ignored_external_collisions:
                    if entry[0] == (bodyA, bodyB):
                        ignored_entry = entry
                        break
            linksA = CollisionChecker.__get_collision_links_for_body(bodyA)
            linksB = CollisionChecker.__get_collision_links_for_body(bodyB)
            link_pairs = CollisionChecker.__build_external_collision_pairs(
                linksA, linksB)
            if ignored_entry is not None:
                for ignore_pair in ignored_entry[1]:
                    if ignore_pair in link_pairs:
                        link_pairs.remove(ignore_pair)
            self.__external_collision_pairs.append(((bodyA, bodyB),
                                                    link_pairs))

    def make_robot_static(self, urdf_id: int):
        """
        Converts a robot body to a static body by updating its entry in
        bodies_information and rebuilding the collision pairs.

        Args:
            urdf_id (int): Unique identifier of the body to be made static.
        """
        for body in self.bodies_information:
            if body['urdf_id'] == urdf_id:
                body['body_type'] = 'static_body'
                self.__update_internal_collision_pairs()
                self.__update_external_collision_pairs()
                break

    def remove_urdf_id(self, urdf_id: int):
        """
        Removes a body (specified by urdf_id) from the
        bodies_information list and rebuilds collision pairs.

        Args:
            urdf_id (int): Unique identifier of the body to be removed.
        """
        self.bodies_information = [body for body in self.bodies_information
                                   if body['urdf_id'] != urdf_id]
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def add_urdf_id(self, urdf_id: int):
        """
        Adds a body (specified by urdf_id) to the bodies_information list
        if not already present. Rebuilds collision pairs after addition.

        Args:
            urdf_id (int): Unique identifier of the body to be added.
        """
        if any(body['urdf_id'] == urdf_id
               for body in self.bodies_information):
            return
        num_bodies = p.getNumBodies()
        if urdf_id < 0 or urdf_id >= num_bodies:
            return  # Invalid URDF id.
        body_info = p.getBodyInfo(urdf_id)
        urdf_name = body_info[1].decode('utf-8')
        links = CollisionChecker.__get_collision_links_for_body(urdf_id)
        if not links:
            return
        body_type = ('robot' if CollisionChecker.has_moving_joint(urdf_id)
                     else 'static_body')
        self.bodies_information.append({
            'urdf_id': urdf_id,
            'urdf_name': urdf_name,
            'body_type': body_type,
            'collision_links': links,
            'collision_pairs':
                CollisionChecker.__build_internal_collision_pairs(links)
        })
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def ignore_external_body_collision(self, body_pair: tuple):
        """
        Ignores all external collisions between the two bodies specified
        in body_pair.

        Args:
            body_pair (tuple): A tuple containing two body IDs. The order
                does not matter.
        """
        normalized_pair = tuple(sorted(body_pair))
        for idx, (bp, _) in enumerate(self.__ignored_external_collisions):
            if bp == normalized_pair:
                self.__ignored_external_collisions[idx] = (
                    normalized_pair, "ALL")
                break
        else:
            self.__ignored_external_collisions.append(
                (normalized_pair, "ALL"))
        self.__update_external_collision_pairs()

    def ignore_external_link_collision(self, body_pair: tuple,
                                       ignored_link_pairs: list):
        """
        Ignores specified external collision link pairs for the given
        body pair.

        Args:
            body_pair (tuple): A tuple containing two body IDs.
            ignored_link_pairs (list): A list of tuples. Each tuple
                contains two link indices (one from each body) that should
                be ignored.
        """
        normalized_pair = tuple(sorted(body_pair))
        for idx, (bp, links) in enumerate(self.__ignored_external_collisions):
            if bp == normalized_pair:
                if links == "ALL":
                    return
                new_links = set(links)
                for lp in ignored_link_pairs:
                    new_links.add(tuple(sorted(lp)))
                self.__ignored_external_collisions[idx] = (
                    normalized_pair, list(new_links))
                break
        else:
            normalized_links = [tuple(sorted(lp))
                                for lp in ignored_link_pairs]
            self.__ignored_external_collisions.append(
                (normalized_pair, normalized_links))
        self.__update_external_collision_pairs()

    def ignore_internal_link_collision(self, urdf_id: int,
                                       ignored_links: tuple):
        """
        Ignores internal collision between any two links specified in
        ignored_links for the given URDF. All unique pairs formed from
        ignored_links will be added to the ignored list.

        Args:
            urdf_id (int): Unique identifier of the robot body.
            ignored_links (tuple): A tuple containing link indices to be
                ignored for internal collision.
        """
        ignored_pairs = list(combinations(sorted(ignored_links), 2))
        for idx, (uid, pairs) in enumerate(self.__ignored_internal_collisions):
            if uid == urdf_id:
                new_pairs = set(pairs)
                for pair in ignored_pairs:
                    new_pairs.add(pair)
                self.__ignored_internal_collisions[idx] = (
                    urdf_id, list(new_pairs))
                break
        else:
            self.__ignored_internal_collisions.append((urdf_id, ignored_pairs))
        self.__update_internal_collision_pairs()

    def clear_ignored_internal_collision(self) -> None:
        """
        Clears all ignored internal collision pairs.

        Returns:
            None
        """
        self.__ignored_internal_collisions = []
        self.__update_internal_collision_pairs()

    def clear_ignored_external_collision(self) -> None:
        """
        Clears all ignored external collision pairs.

        Returns:
            None
        """
        self.__ignored_external_collisions = []
        self.__update_external_collision_pairs()

    def is_collision_free(self) -> bool:
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
            bool: True if no internal collisions are detected;
            False otherwise.
        """
        if self.enable_internal_collision:
            for urdf_id, pairs in self.__internal_collision_pairs:
                for linkA, linkB in pairs:
                    if CollisionChecker.simple_collision(
                            urdf_id, urdf_id, MAX_DISTANCE_INTERNAL,
                            linkA, linkB):
                        return False
        return True

    def check_external_collisions(self) -> bool:
        """
        Checks for collisions between link pairs from different bodies.

        Returns:
            bool: True if no external collisions are detected;
            False otherwise.
        """
        if self.enable_external_collision:
            for (bodyA, bodyB), link_pairs in self.__external_collision_pairs:
                for linkA, linkB in link_pairs:
                    if CollisionChecker.simple_collision(
                            bodyA, bodyB, self.max_distance_external,
                            linkA, linkB):
                        return False
        return True

    def set_safe_state(self) -> None:
        """
        Sets the current collision configuration as the safe state.
        This method updates the collision pair lists without filtering,
        records the current collisions as ignored, and then updates the
        lists to exclude these safe collisions.

        Returns:
            None
        """
        self.__update_internal_collision_pairs(ignore_collision=False)
        self.__update_external_collision_pairs(ignore_collision=False)

        self.__ignored_internal_collisions = self.get_internal_collisions()
        self.__ignored_external_collisions = self.get_global_external_collisions()
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def get_internal_collisions(self) -> list:
        """
        Identifies and returns internal collision pairs that are
        colliding within each robot.

        Returns:
            list: A list of tuples, each containing a urdf_id and a list of
            colliding link pairs.
        """
        internal_collisions = []
        for entry in self.__internal_collision_pairs:
            link_pair_collisions = []
            for linkA, linkB in entry[1]:
                if CollisionChecker.simple_collision(
                        entry[0], entry[0], MAX_DISTANCE_INTERNAL,
                        linkA, linkB):
                    link_pair_collisions.append((linkA, linkB))
            if link_pair_collisions:
                internal_collisions.append(
                    (entry[0], link_pair_collisions))
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
        for (bodyA, bodyB), link_pairs in self.__external_collision_pairs:
            colliding_links = []
            for linkA, linkB in link_pairs:
                if CollisionChecker.simple_collision(
                        bodyA, bodyB, self.max_distance_external,
                        linkA, linkB):
                    colliding_links.append((linkA, linkB))
            if colliding_links:
                global_external_collisions.append(
                    ((bodyA, bodyB), colliding_links))
        return global_external_collisions

    def get_global_distance(self, distance: float) -> float:
        """
        Determines the smallest distance across all external collision pairs.

        Args:
            distance (float): An initial maximum distance threshold.

        Returns:
            float: The smallest distance found among the external pairs.
        """
        min_distance = distance
        for (bodyA, bodyB), _ in self.__external_collision_pairs:
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
        for (bodies, link_pairs) in self.__external_collision_pairs:
            if bodies == pair_key:
                min_distance = distance
                for linkA, linkB in link_pairs:
                    curr_dist = CollisionChecker.get_distance(
                        bodyA, bodyB, distance, linkA, linkB)
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
        linksA = CollisionChecker.__get_collision_links_for_body(bodyA)
        linksB = CollisionChecker.__get_collision_links_for_body(bodyB)
        ext_pairs = CollisionChecker.__build_external_collision_pairs(
            linksA, linksB)
        external_collisions = []
        for linkA, linkB in ext_pairs:
            if CollisionChecker.simple_collision(
                    bodyA, bodyB, max_distance, linkA, linkB):
                external_collisions.append((linkA, linkB))
        return external_collisions

    @staticmethod
    def has_moving_joint(urdf_id: int) -> bool:
        """
        Determines if the specified body has at least one movable joint.

        Args:
            urdf_id (int): Unique identifier of the body.

        Returns:
            bool: True if at least one joint is movable; False otherwise.
        """
        num_joints = p.getNumJoints(urdf_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(urdf_id, j)
            if joint_info[2] != p.JOINT_FIXED:
                return True
        return False

    @staticmethod
    def __get_collision_links_for_body(urdf_id: int) -> list:
        """
        Retrieves a sorted list of link indices that have collision geometry
        for the given body. This includes the base link.

        Args:
            urdf_id (int): Unique identifier of the body.

        Returns:
            list: Sorted list of link indices with collision shapes.
        """
        links_with_collision = []
        if p.getCollisionShapeData(urdf_id, BASE_LINK):
            links_with_collision.append(BASE_LINK)
        num_joints = p.getNumJoints(urdf_id)
        for link in range(num_joints):
            if p.getCollisionShapeData(urdf_id, link):
                links_with_collision.append(link)
        return sorted(links_with_collision)

    @staticmethod
    def __build_internal_collision_pairs(links: list) -> list:
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
    def __build_external_collision_pairs(links1: list,
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
            bool: True if a collision (or contact within max_distance)
            is detected; False otherwise.
        """
        contacts = p.getClosestPoints(bodyA, bodyB, max_distance,
                                      linkIndexA=linkA,
                                      linkIndexB=linkB)
        return len(contacts) > 0

    @staticmethod
    def get_distance(bodyA: int, bodyB: int, distance: float,
                     linkA: int, linkB: int) -> float:
        """
        Computes the closest distance between two specified links from
        two bodies using PyBullet's getClosestPoints.

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
        contacts = p.getClosestPoints(bodyA, bodyB, distance,
                                      linkIndexA=linkA,
                                      linkIndexB=linkB)
        if contacts:
            return min(contact[8] for contact in contacts)
        return distance
