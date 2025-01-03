import numpy as np
import pybullet as p


class CollisionChecker:

    def __init__(self):
        pass

    def get_self_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
        moving_links = get_moving_links(body, joints)
        fixed_links = list(set(get_joints(body)) - set(moving_links))
        check_link_pairs = list(product(moving_links, fixed_links))
        if only_moving:
            check_link_pairs.extend(get_moving_pairs(body, joints))
        else:
            check_link_pairs.extend(combinations(moving_links, 2))
        check_link_pairs = list(
            filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs))
        check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                    (pair[::-1] not in disabled_collisions), check_link_pairs))
        return check_link_pairs

    def get_moving_links(body, joints):
        moving_links = set()
        for joint in joints:
            link = child_link_from_joint(joint)
            if link not in moving_links:
                moving_links.update(get_link_subtree(body, link))
        return list(moving_links)