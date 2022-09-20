from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.raycaster import RayCaster
from pybullet_industrial.robot_base import RobotBase


class Remover(RayCaster):

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 remover_properties: Dict, coupled_robot: RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):
        """Special Remover Tool which can remove objects from the simulation.

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position (np.array): the position at which the tool should be spawned
            start_orientation (np.array): the orientation at which the tool should be spawned
            remover_properties(Dict): A dictionary containing the properties of the extrusion head.
                                       During initialization only 'material' has to be set.
                                       Default Values are:
                                       'opening angle':0,'number of rays':1,
                                       'maximum distance':1
            coupled_robot (RobotBase, optional): A pybullet_industrial.RobotBase object if
                                                 the robot is coupled from the start.
                                                 Defaults to None.
            tcp_frame (str, optional): The name of the urdf_link
                                       describing the tool center point.
                                       Defaults to None in which case the last link is used.
            connector_frame (str, optional): The name of the urdf_link
                                             at which a robot connects.
                                             Defaults to None in which case the base link is used.
        """

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.change_properties(remover_properties)

    def remove(self, tcp_frame=None):
        """Removes simulation elements from the simulation environment.

        Args:
            tcp_frame (str, optional): the name of the link from which to remove the material.
                                       Defaults to None in which case the default tcp is used
        """
        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation)

        removed_objects = []
        for ray_intersection in ray_cast_results:
            if ray_intersection[0] != -1:
                p.removeBody(ray_intersection[0])
                removed_objects.append(ray_intersection[0])
        return removed_objects
