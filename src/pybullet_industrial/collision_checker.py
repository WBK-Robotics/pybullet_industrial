import pybullet as p
from itertools import product, combinations
from collections import namedtuple
from pybullet_industrial import RobotBase

BASE_LINK = -1
MAX_DISTANCE = 0.0


class CollisionChecker:
    """Checks collisions for a robot against obstacles and for self-
    collisions.

    This class verifies that the robot does not collide with the environment
    or itself. The user can specify a maximum distance threshold for collision
    queries and, separately, for clearance queries, a dictionary mapping
    obstacles to custom query distances.

    Args:
        robot (RobotBase): The robot model and its kinematics.
        obstacles (list): List of obstacle IDs present in the environment.
        self_collisions (bool, optional): Whether to enable self-collision
            checking. Defaults to True.
        allow_collision_links (list, optional): List of allowed collision link
            pairs to ignore during checks. Defaults to [].
        max_distance (float, optional): The maximum distance for considering a
            collision. Also used as the default query distance if a custom one is
            not provided. Defaults to MAX_DISTANCE.
        clearance_obstacles (dict, optional): A dictionary mapping each obstacle
            (ID) to its custom query distance. If not provided, it defaults to a
            dictionary with each obstacle mapped to max_distance.
    """
    def __init__(self, robot: RobotBase, obstacles,
                 self_collisions=True, allow_collision_links=[],
                 max_distance=MAX_DISTANCE, clearance_obstacles=None):
        self.robot = robot
        self.obstacles = obstacles
        self.self_collisions = self_collisions
        self.allow_collision_links = allow_collision_links
        self.max_distance = max_distance
        if clearance_obstacles is None:
            # Default: every obstacle uses the same query distance.
            self.clearance_obstacles = {obs: max_distance for obs in obstacles}
        else:
            self.clearance_obstacles = clearance_obstacles
        self.update_collision_settings()

    def update_collision_settings(self):
        """Updates collision settings based on the robot and obstacles.

        - Determines link pairs for self-collision checks from the robot's
          moving joints.
        - Filters out allowed collision link pairs.
        - Defines robot-obstacle pairs for both environment collision checks and
          clearance queries.
        """
        # Self-collision checks.
        self.check_link_pairs = (
            self.get_self_link_pairs(self.robot.urdf,
                                     self.robot.get_moveable_joints()[1])
            if self.self_collisions else []
        )
        self.check_link_pairs = [
            pair for pair in self.check_link_pairs
            if pair not in self.allow_collision_links
        ]
        # Define moving links.
        moving_links = frozenset(
            item for item in self.get_moving_links(
                self.robot.urdf, self.robot.get_moveable_joints()[1])
            if item not in self.allow_collision_links
        )
        moving_bodies = [(self.robot.urdf, moving_links)]
        # Pairs for collision checking with obstacles.
        self.check_body_pairs = list(product(moving_bodies, self.obstacles))
        # Pairs for clearance checking.
        self.check_clearance_body_pairs = list(
            product(moving_bodies, self.clearance_obstacles.keys())
        )

    def set_obstacles(self, obstacles):
        """Updates the list of obstacles and reconfigures collision detection.

        Args:
            obstacles (list): List of new obstacle IDs.
        """
        self.obstacles = obstacles
        self.update_collision_settings()

    def check_collision(self):
        """Checks if the current state of the robot is collision-free.

        Performs self-collision and environment collision checks.

        Returns:
            bool: True if no collisions are detected, False otherwise.
        """
        for link1, link2 in self.check_link_pairs:
            if self.pairwise_link_collision(self.robot.urdf, link1,
                                            self.robot.urdf, link2,
                                            max_distance=self.max_distance):
                return False
        for body1, body2 in self.check_body_pairs:
            if self.pairwise_collision(body1, body2,
                                       max_distance=self.max_distance):
                return False
        return True

    def get_collision_links(self):
        """Returns a list of robot link pairs currently in collision.

        Returns:
            list: A list of tuples representing colliding link pairs.
        """
        collision_links = []
        for link1, link2 in self.check_link_pairs:
            if self.pairwise_link_collision(self.robot.urdf, link1,
                                            self.robot.urdf, link2,
                                            max_distance=self.max_distance):
                collision_links.append((link1, link2))
        return collision_links

    def get_max_distance(self):
        """Returns the minimum distance between the robot and any obstacle.

        Iterates over the clearance body pairs (robot's moving links and each
        clearance obstacle) and uses the helper function body_distance() to
        compute the distance for each pair. The smallest distance is returned.
        If no contact points are found, returns the maximum custom query
        distance among obstacles.

        Returns:
            float: The minimum distance found between the robot and an obstacle.
        """
        min_dist = float('inf')
        for (body1, links1), obs in self.check_clearance_body_pairs:
            query_distance = self.clearance_obstacles[obs]
            d = self.body_distance(body1, links1, obs, query_distance)
            if d < min_dist:
                if d <= 0:
                    return 0
                min_dist = d
        if min_dist == float('inf'):
            return max(self.clearance_obstacles.values())
        return min_dist

    # -------------------------------------------------------------------------
    # Static Helper Methods for Collision Checking
    # -------------------------------------------------------------------------
    @staticmethod
    def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK,
                                max_distance=MAX_DISTANCE):
        """Checks for collision between two specific links of two bodies.

        Args:
            body1: First body ID.
            link1: Link index of the first body.
            body2: Second body ID.
            link2: Link index of the second body (defaults to base link).
            max_distance (float): Maximum distance for collision proximity.

        Returns:
            bool: True if the links are in collision, False otherwise.
        """
        return len(p.getClosestPoints(
            bodyA=body1, bodyB=body2, distance=max_distance,
            linkIndexA=link1, linkIndexB=link2)) != 0

    @staticmethod
    def pairwise_collision(body1, body2, **kwargs):
        """Checks for collision between two bodies, expanding their links if
        needed.

        Args:
            body1: First body ID or tuple (body ID, list of links).
            body2: Second body ID or tuple (body ID, list of links).

        Returns:
            bool: True if any link pairs are in collision, False otherwise.
        """
        if isinstance(body1, tuple) or isinstance(body2, tuple):
            body1, links1 = CollisionChecker.expand_links(body1)
            body2, links2 = CollisionChecker.expand_links(body2)
            return CollisionChecker.any_link_pair_collision(
                body1, links1, body2, links2, **kwargs)
        return CollisionChecker.body_collision(body1, body2, **kwargs)

    @staticmethod
    def expand_links(body):
        """Expands a body ID to include all its links if not explicitly provided.

        Args:
            body: A body ID or tuple (body ID, list of links).

        Returns:
            tuple: A tuple (body ID, list of links).
        """
        body, links = body if isinstance(body, tuple) else (body, None)
        if links is None:
            links = CollisionChecker.get_all_links(body)
        return body, links

    @staticmethod
    def any_link_pair_collision(body1, links1, body2, links2=None, **kwargs):
        """Checks for collisions between any pair of links from two bodies.

        Args:
            body1: First body ID.
            links1: List of links for the first body.
            body2: Second body ID.
            links2: List of links for the second body (defaults to all links).

        Returns:
            bool: True if any link pair is in collision, False otherwise.
        """
        if links1 is None:
            links1 = CollisionChecker.get_all_links(body1)
        if links2 is None:
            links2 = CollisionChecker.get_all_links(body2)
        for link1, link2 in product(links1, links2):
            if (body1 == body2) and (link1 == link2):
                continue
            if CollisionChecker.pairwise_link_collision(
                    body1, link1, body2, link2, **kwargs):
                return True
        return False

    @staticmethod
    def body_collision(body1, body2, max_distance=MAX_DISTANCE):
        """Checks for collision between two bodies as wholes.

        Args:
            body1: First body ID.
            body2: Second body ID.
            max_distance (float): Maximum distance for collision proximity.

        Returns:
            bool: True if the bodies are in collision, False otherwise.
        """
        return len(p.getClosestPoints(
            bodyA=body1, bodyB=body2, distance=max_distance)) != 0

    @staticmethod
    def body_distance(body1, links1, obs, query_distance):
        """Calculates the minimum distance between the given robot body (with its
        moving links) and an obstacle using a custom query distance.

        Args:
            body1: The robot body ID.
            links1: The set or list of moving links for the robot.
            obs: The obstacle ID.
            query_distance (float): The custom query distance for this obstacle.

        Returns:
            float: The minimum distance found between any link and the obstacle.
        """
        min_d = float('inf')
        for link in links1:
            pts = p.getClosestPoints(
                bodyA=body1, bodyB=obs,
                distance=query_distance, linkIndexA=link,
                linkIndexB=BASE_LINK
            )
            for pt in pts:
                # Always assume that the contact distance is stored at index 8.
                d = pt[8]
                if d < min_d:
                    min_d = d
        if min_d == float('inf'):
            return query_distance
        return min_d

    # -------------------------------------------------------------------------
    # Static Helper Methods for Robot Structure and Kinematics
    # -------------------------------------------------------------------------
    @staticmethod
    def get_self_link_pairs(body, joints, disabled_collisions=set(),
                            only_moving=True):
        """Retrieves pairs of links for self-collision checks.

        Args:
            body: Body ID of the robot.
            joints: List of joint indices defining the robot's kinematics.
            disabled_collisions (set): Link pairs to ignore.
            only_moving (bool): Whether to include only moving links.

        Returns:
            list: List of link pairs to check.
        """
        moving_links = CollisionChecker.get_moving_links(body, joints)
        fixed_links = list(set(CollisionChecker.get_joints(body)) - set(moving_links))
        check_link_pairs = list(product(moving_links, fixed_links))
        if only_moving:
            check_link_pairs.extend(
                CollisionChecker.get_moving_pairs(body, joints))
        else:
            check_link_pairs.extend(combinations(moving_links, 2))
        check_link_pairs = list(filter(
            lambda pair: not CollisionChecker.are_links_adjacent(body, *pair),
            check_link_pairs))
        check_link_pairs = list(filter(
            lambda pair: (pair not in disabled_collisions) and
            (pair[::-1] not in disabled_collisions),
            check_link_pairs))
        return check_link_pairs

    @staticmethod
    def get_moving_links(body, joints):
        """Retrieves all moving links influenced by the given joints.

        Args:
            body: Body ID of the robot.
            joints: List of joint indices defining the robot's kinematics.

        Returns:
            list: List of indices of moving links.
        """
        moving_links = set()
        for joint in joints:
            link = CollisionChecker.child_link_from_joint(joint)
            if link not in moving_links:
                moving_links.update(
                    CollisionChecker.get_link_subtree(body, link))
        return list(moving_links)

    @staticmethod
    def get_moving_pairs(body, moving_joints):
        """Retrieves pairs of moving links for self-collision checks.

        Args:
            body: Body ID of the robot.
            moving_joints: List of joint indices influencing the robot's moving
            links.

        Yields:
            tuple: A pair of moving link indices.
        """
        moving_links = CollisionChecker.get_moving_links(body, moving_joints)
        for link1, link2 in combinations(moving_links, 2):
            ancestors1 = set(
                CollisionChecker.get_joint_ancestors(body, link1)
            ) & set(moving_joints)
            ancestors2 = set(
                CollisionChecker.get_joint_ancestors(body, link2)
            ) & set(moving_joints)
            if ancestors1 != ancestors2:
                yield link1, link2

    @staticmethod
    def child_link_from_joint(joint):
        return joint

    @staticmethod
    def get_num_joints(body):
        return p.getNumJoints(body)

    @staticmethod
    def get_joints(body):
        return list(range(CollisionChecker.get_num_joints(body)))

    @staticmethod
    def get_all_links(body):
        return [BASE_LINK] + list(CollisionChecker.get_joints(body))

    @staticmethod
    def get_link_parent(body, link):
        if link == BASE_LINK:
            return None
        return CollisionChecker.get_joint_info(body, link).parentIndex

    @staticmethod
    def get_all_link_parents(body):
        return {link: CollisionChecker.get_link_parent(body, link)
                for link in CollisionChecker.get_joints(body)}

    @staticmethod
    def get_all_link_children(body):
        children = {}
        for child, parent in CollisionChecker.get_all_link_parents(body).items():
            if parent not in children:
                children[parent] = []
            children[parent].append(child)
        return children

    @staticmethod
    def get_link_children(body, link):
        children = CollisionChecker.get_all_link_children(body)
        return children.get(link, [])

    @staticmethod
    def get_link_ancestors(body, link):
        parent = CollisionChecker.get_link_parent(body, link)
        if parent is None:
            return []
        return CollisionChecker.get_link_ancestors(body, parent) + [parent]

    @staticmethod
    def get_joint_ancestors(body, joint):
        link = CollisionChecker.child_link_from_joint(joint)
        return CollisionChecker.get_link_ancestors(body, link) + [link]

    @staticmethod
    def get_link_descendants(body, link, test=lambda l: True):
        descendants = []
        for child in CollisionChecker.get_link_children(body, link):
            if test(child):
                descendants.append(child)
                descendants.extend(
                    CollisionChecker.get_link_descendants(body, child, test=test)
                )
        return descendants

    @staticmethod
    def get_link_subtree(body, link, **kwargs):
        return [link] + CollisionChecker.get_link_descendants(body, link, **kwargs)

    @staticmethod
    def are_links_adjacent(body, link1, link2):
        return (CollisionChecker.get_link_parent(body, link1) == link2) or \
               (CollisionChecker.get_link_parent(body, link2) == link1)

    # -------------------------------------------------------------------------
    # Joint Info
    # -------------------------------------------------------------------------
    JointInfo = namedtuple(
        'JointInfo', ['jointIndex', 'jointName', 'jointType',
                      'qIndex', 'uIndex', 'flags', 'jointDamping',
                      'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                      'jointMaxForce', 'jointMaxVelocity', 'linkName',
                      'jointAxis', 'parentFramePos', 'parentFrameOrn',
                      'parentIndex']
    )

    @staticmethod
    def get_joint_info(body, joint):
        return CollisionChecker.JointInfo(*p.getJointInfo(body, joint))
