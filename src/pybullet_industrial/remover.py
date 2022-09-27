from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.raycaster import RayCaster
from pybullet_industrial.robot_base import RobotBase


class Remover(RayCaster):
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

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 remover_properties: Dict, coupled_robot: RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.change_properties(remover_properties)

    def calculate_process_force(self, ray_cast_results, tcp_frame=None):
        """Function calculating the process force of the milling tool.

        Args:
            ray_cast_results (list): The result of a batch ray cast
                                       as performed by the remove function
            tcp_frame (str): The name of the tool center point frame. Defaults to None.

        Returns:
            np.array: a 3 dimensional array containing the process force in the x,y,z direction
        """

        return np.zeros(3)

    def remove(self, tcp_frame=None):
        """Removes simulation elements from the simulation environment.

        Args:
            tcp_frame (str, optional): the name of the link from which to remove the material.
                                       Defaults to None in which case the default tcp is used
        """
        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation)

        process_force = self.calculate_process_force(
            ray_cast_results, tcp_frame)
        self.apply_tcp_force(process_force, tcp_frame)

        removed_objects = []
        for ray_intersection in ray_cast_results:
            if ray_intersection[0] != -1:
                p.removeBody(ray_intersection[0])
                removed_objects.append(ray_intersection[0])
        return removed_objects
