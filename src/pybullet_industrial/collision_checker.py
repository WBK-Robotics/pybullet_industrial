import pybullet as p
from itertools import combinations, product

BASE_LINK = -1
MAX_DISTANCE_INTERNAL = 0.0


class CollisionChecker:
    """
    Manages collision geometry for bodies in a PyBullet sim and
    categorizes them based on joint mobility.

    Args:
        body_ids (list): List of body IDs to consider. If None, all
            bodies are processed.
        max_distance_external (float): Maximum distance threshold for
            external collision detection.
        enable_internal_collision (bool): Flag to enable internal
            collision checking.
        enable_external_collision (bool): Flag to enable external
            collision checking.
    """

    def __init__(self, body_ids: list = None,
                 max_distance_external: float = 0.0,
                 enable_internal_collision: bool = True,
                 enable_external_collision: bool = True):

        self.max_distance_external = max_distance_external
        self._bodies_information = []
        self._ignored_internal_collisions = []
        self._ignored_external_collisions = []
        self._internal_collision_pairs = []
        self._external_collision_pairs = []
        self.enable_internal_collision = enable_internal_collision
        self.enable_external_collision = enable_external_collision

        self.__build_bodies_information(body_ids)
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def __build_bodies_information(self, body_ids: list) -> None:
        """
        Iterates through all bodies in the sim and collects collision
        geometry information. If a list of body IDs is provided, only those
        bodies are processed.

        For each body, determines whether it is a 'robot' (if it has at least
        one movable joint) or a 'static_body'. It also computes internal
        collision pairs.

        Additionally, the body name is decoded from bytes and stored as
        'body_name'.

        Args:
            body_ids (list): List of body IDs to process.

        Returns:
            None
        """
        num_bodies = p.getNumBodies()
        for body_id in range(num_bodies):
            if body_ids is not None and body_id not in body_ids:
                continue
            body_info = p.getBodyInfo(body_id)
            body_name = body_info[1].decode('utf-8')
            links = CollisionChecker.__get_body_collision_links(body_id)
            if not links:
                continue
            body_type = ('robot'
                         if CollisionChecker.has_moving_joint(body_id)
                         else 'static_body')
            self._bodies_information.append({
                'body_id': body_id,
                'body_name': body_name,
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
        in _internal_collision_pairs as a list of tuples:
        (body_id, [list of valid link pairs]).

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.

        Returns:
            None
        """
        self._internal_collision_pairs = []
        for body in self._bodies_information:
            if body['body_type'] == 'robot':
                valid_pairs = body['collision_pairs'].copy()
                if ignore_collision:
                    for ignored in self._ignored_internal_collisions:
                        if ignored[0] == body['body_id']:
                            for pair in ignored[1]:
                                normalized_pair = tuple(sorted(pair))
                                if normalized_pair in valid_pairs:
                                    valid_pairs.remove(normalized_pair)
                self._internal_collision_pairs.append(
                    (body['body_id'], valid_pairs)
                )

    def __update_external_collision_pairs(
            self, ignore_collision: bool = True) -> None:
        """
        Updates the list of external collision pairs between bodies.
        Each entry in _external_collision_pairs is a tuple:
        ((bodyA, bodyB), [list of link pairs]).

        For each body pair, if the pair is in the ignored list, the
        corresponding link pairs are removed from the complete Cartesian
        product of their collision links.

        Args:
            ignore_collision (bool): Whether to filter out ignored pairs.

        Returns:
            None
        """
        body_ids = [body['body_id'] for body in self._bodies_information]
        potential_body_pairs = list(combinations(body_ids, 2))
        self._external_collision_pairs = []
        for bodyA, bodyB in potential_body_pairs:
            ignored_entry = None
            if ignore_collision:
                for entry in self._ignored_external_collisions:
                    if entry[0] == (bodyA, bodyB):
                        ignored_entry = entry
                        break
            linksA = CollisionChecker.__get_body_collision_links(bodyA)
            linksB = CollisionChecker.__get_body_collision_links(bodyB)
            link_pairs = CollisionChecker.__build_external_collision_pairs(
                linksA, linksB)
            if ignored_entry is not None:
                if ignored_entry[1] == "ALL":
                    link_pairs = []
                else:
                    for ignore_pair in ignored_entry[1]:
                        if ignore_pair in link_pairs:
                            link_pairs.remove(ignore_pair)
            self._external_collision_pairs.append(((bodyA, bodyB),
                                                   link_pairs))

    def make_robot_static(self, body_id: int):
        """
        Converts a robot body to a static body by updating its entry in
        bodies_information and rebuilding the collision pairs.

        Args:
            body_id (int): Unique identifier of the body to be made static.
        """
        for body in self._bodies_information:
            if body['body_id'] == body_id:
                body['body_type'] = 'static_body'
                self.__update_internal_collision_pairs()
                self.__update_external_collision_pairs()
                break

    def remove_body_id(self, body_id: int):
        """
        Removes a body (specified by body_id) from the bodies_information list
        and rebuilds collision pairs.

        Args:
            body_id (int): Unique identifier of the body to be removed.
        """
        self._bodies_information = [body for body in self._bodies_information
                                    if body['body_id'] != body_id]
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def remove_link_id(self, body_id: int, link_id: int) -> None:
        """
        Removes the specified link from the collision information
        for a given body and updates collision pairs.

        Args:
            body_id (int): Unique identifier of the body.
            link_id (int): Link index to be removed.
        """
        for body in self._bodies_information:
            if body['body_id'] == body_id:
                if link_id in body['collision_links']:
                    body['collision_links'].remove(link_id)
                    body['collision_pairs'] = (
                        CollisionChecker.__build_internal_collision_pairs(
                            body['collision_links']
                        )
                    )
                    break
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def add_body_id(self, body_id: int):
        """
        Adds a body (specified by body_id) to the bodies_information list
        if not already present. Rebuilds collision pairs after addition.

        Args:
            body_id (int): Unique identifier of the body to be added.
        """
        if any(body['body_id'] == body_id
               for body in self._bodies_information):
            return
        num_bodies = p.getNumBodies()
        if body_id < 0 or body_id >= num_bodies:
            return  # Invalid body id.
        body_info = p.getBodyInfo(body_id)
        body_name = body_info[1].decode('utf-8')
        links = CollisionChecker.__get_body_collision_links(body_id)
        if not links:
            return
        body_type = ('robot' if CollisionChecker.has_moving_joint(body_id)
                     else 'static_body')
        self._bodies_information.append({
            'body_id': body_id,
            'body_name': body_name,
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
        for idx, (bp, _) in enumerate(self._ignored_external_collisions):
            if bp == normalized_pair:
                self._ignored_external_collisions[idx] = (
                    normalized_pair, "ALL")
                break
        else:
            self._ignored_external_collisions.append(
                (normalized_pair, "ALL"))
        self.__update_external_collision_pairs()

    def ignore_external_link_collision(self, body_pair: tuple,
                                       ignored_link_pairs: list):
        """
        Ignores specified external collision link pairs for the given
        body pair.

        Args:
            body_pair (tuple): A tuple containing two body IDs.
            ignored_link_pairs (list): A list of tuples. Each tuple contains
                two link indices (one from each body) that should be ignored.
        """
        normalized_pair = tuple(sorted(body_pair))
        for idx, (bp, links) in enumerate(self._ignored_external_collisions):
            if bp == normalized_pair:
                if links == "ALL":
                    return
                new_links = set(links)
                for lp in ignored_link_pairs:
                    new_links.add(tuple(sorted(lp)))
                self._ignored_external_collisions[idx] = (
                    normalized_pair, list(new_links))
                break
        else:
            normalized_links = [tuple(sorted(lp))
                                for lp in ignored_link_pairs]
            self._ignored_external_collisions.append(
                (normalized_pair, normalized_links))
        self.__update_external_collision_pairs()

    def ignore_internal_link_collision(self, body_id: int,
                                       ignored_links: tuple):
        """
        Ignores internal collision between any two links specified in
        ignored_links for the given body. All unique pairs formed from
        ignored_links will be added to the ignored list.

        Args:
            body_id (int): Unique identifier of the robot body.
            ignored_links (tuple): A tuple containing link indices to be
                ignored for internal collision.
        """
        ignored_pairs = list(combinations(sorted(ignored_links), 2))
        for idx, (bid, pairs) in enumerate(self._ignored_internal_collisions):
            if bid == body_id:
                new_pairs = set(pairs)
                for pair in ignored_pairs:
                    new_pairs.add(pair)
                self._ignored_internal_collisions[idx] = (
                    body_id, list(new_pairs))
                break
        else:
            self._ignored_internal_collisions.append((body_id, ignored_pairs))
        self.__update_internal_collision_pairs()

    def clear_ignored_internal_collision(self) -> None:
        """
        Clears all ignored internal collision pairs.

        Returns:
            None
        """
        self._ignored_internal_collisions = []
        self.__update_internal_collision_pairs()

    def clear_ignored_external_collision(self) -> None:
        """
        Clears all ignored external collision pairs.

        Returns:
            None
        """
        self._ignored_external_collisions = []
        self.__update_external_collision_pairs()

    def is_collision_free(self) -> bool:
        """
        Checks for collisions within individual robot bodies and between
        different bodies.

        Returns:
            bool: True if no collisions are detected; False otherwise.
        """
        if self.enable_internal_collision:
            if not self.is_internal_collision_free():
                return False
        if self.enable_external_collision:
            if not self.is_external_collision_free():
                return False
        return True

    def is_internal_collision_free(self) -> bool:
        """
        Checks for collisions between internal link pairs within each robot.

        Returns:
            bool: True if no internal collisions are detected;
            False otherwise.
        """
        if self.enable_internal_collision:
            for body_id, pairs in self._internal_collision_pairs:
                for linkA, linkB in pairs:
                    if CollisionChecker.simple_collision(
                            body_id, body_id, MAX_DISTANCE_INTERNAL,
                            linkA, linkB):
                        return False
        return True

    def is_external_collision_free(self) -> bool:
        """
        Checks for collisions between link pairs from different bodies.

        Returns:
            bool: True if no external collisions are detected;
            False otherwise.
        """
        if self.enable_external_collision:
            for (bodyA, bodyB), link_pairs in self._external_collision_pairs:
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

        self._ignored_internal_collisions = self.get_internal_collisions()
        self._ignored_external_collisions = self.get_external_collisions()
        self.__update_internal_collision_pairs()
        self.__update_external_collision_pairs()

    def get_internal_collisions(self) -> list:
        """
        Identifies and returns internal collision pairs that are
        colliding within each robot.

        Returns:
            list: A list of tuples, each containing a body_id and a list of
            colliding link pairs.
        """
        internal_collisions = []
        for entry in self._internal_collision_pairs:
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

    def get_external_collisions(self) -> list:
        """
        Checks all external collision pairs between bodies and returns those
        that are colliding.

        Returns:
            list: A list of tuples where each tuple contains a body pair and a
            list of colliding link pairs.
        """
        global_external_collisions = []
        for (bodyA, bodyB), link_pairs in self._external_collision_pairs:
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

    def get_external_distance(self, distance: float):
        """
        Determines the smallest distance across all external collision pairs.

        Args:
            distance (float): An initial maximum distance threshold.

        Returns:
            Optional[float]: The smallest distance found among the external
            pairs, or None if no collision link pair is found.
        """
        result = None
        for (bodyA, bodyB), _ in self._external_collision_pairs:
            body_distance = self.get_body_distance(bodyA, bodyB, distance)
            if body_distance is not None:
                if result is None or body_distance < result:
                    result = body_distance
                distance = result  # Update threshold to current minimum
        return result

    def get_body_distance(self, bodyA: int, bodyB: int, distance: float):
        """
        Determines the smallest distance between any collision pair
        (i.e. link pair) for a given pair of bodies by inspecting
        external collision pairs. If no collision link pair is found,
        None is returned.

        Args:
            bodyA (int): Unique identifier of the first body.
            bodyB (int): Unique identifier of the second body.
            distance (float): The initial maximum distance threshold.

        Returns:
            Optional[float]: The smallest distance found among the pairs,
            or None if no collision link pair is found.
        """
        pair_key = tuple(sorted([bodyA, bodyB]))
        for bodies, link_pairs in self._external_collision_pairs:
            if bodies == pair_key:
                min_distance = None
                for linkA, linkB in link_pairs:
                    curr_dist = CollisionChecker.get_distance(
                        bodyA, bodyB, distance, linkA, linkB)
                    if curr_dist is None:
                        continue
                    if min_distance is None or curr_dist < min_distance:
                        min_distance = curr_dist
                return min_distance
        return None

    @staticmethod
    def get_bodies_external_collisions(bodyA: int, bodyB: int,
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
        linksA = CollisionChecker.__get_body_collision_links(bodyA)
        linksB = CollisionChecker.__get_body_collision_links(bodyB)
        ext_pairs = CollisionChecker.__build_external_collision_pairs(
            linksA, linksB)
        external_collisions = []
        for linkA, linkB in ext_pairs:
            if CollisionChecker.simple_collision(
                    bodyA, bodyB, max_distance, linkA, linkB):
                external_collisions.append((linkA, linkB))
        return external_collisions

    @staticmethod
    def has_moving_joint(body_id: int) -> bool:
        """
        Determines if the specified body has at least one movable joint.

        Args:
            body_id (int): Unique identifier of the body.

        Returns:
            bool: True if at least one joint is movable; False otherwise.
        """
        num_joints = p.getNumJoints(body_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(body_id, j)
            if joint_info[2] != p.JOINT_FIXED:
                return True
        return False

    @staticmethod
    def __get_body_collision_links(body_id: int) -> list:
        """
        Retrieves a sorted list of link indices that have collision geometry
        for the given body. This includes the base link.

        Args:
            body_id (int): Unique identifier of the body.

        Returns:
            list: Sorted list of link indices with collision shapes.
        """
        links_with_collision = []
        if p.getCollisionShapeData(body_id, BASE_LINK):
            links_with_collision.append(BASE_LINK)
        num_joints = p.getNumJoints(body_id)
        for link in range(num_joints):
            if p.getCollisionShapeData(body_id, link):
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
        return None
