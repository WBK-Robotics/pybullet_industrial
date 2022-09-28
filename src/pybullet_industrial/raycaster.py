from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool
from pybullet_industrial.robot_base import RobotBase


class RayCaster(EndeffectorTool):
    """Special Endeffector Tool which can cast rays. Base for Extruder and Remover classes.

    Args:
        urdf_model (str): A valid path to a urdf file describint the tool geometry
        start_position (np.array): the position at which the tool should be spawned
        start_orientation (np.array): the orientation at which the tool should be spawned
        raycast_properties(Dict): A dictionary containing the properties of the extrusion head.
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
                 raycast_properties: Dict, coupled_robot: RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.properties = {'opening angle': 0,
                           'number of rays': 1,
                           'maximum distance': 1}
        if raycast_properties is not None:
            self.change_properties(raycast_properties)

    def change_properties(self, new_properties: Dict):
        """Allows retroactive changes to the ray casting properties.


        Args:
            new_properties (Dict): A dictionary containing values and keys that
                                   should be changed

        Raises:
            KeyError: If a key is not a valid property
        """
        for key in new_properties:
            if not key in self.properties:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.properties.keys()))
            self.properties[key] = new_properties[key]

    def cast_rays(self, position: np.array, orientation: np.array):
        """Randomly casts rays withn a given range from a specified position and orientation

        Args:
            position (np.array): start position of the raycast
            orientation (np.array): start orientation of the raycast

        Returns:
            List: The result of a raycast
        """
        opening_angle = self.properties['opening angle']
        number_of_rays = self.properties['number of rays']
        ray_length = self.properties['maximum distance']
        ray_start_pos = []
        ray_end_pos = []

        phi = np.random.uniform(-np.pi, np.pi, number_of_rays)
        theta = np.random.uniform(-0.5*opening_angle,
                                  0.5*opening_angle, number_of_rays)

        ray_directions = np.array([np.sin(theta) * np.cos(phi),
                                   np.sin(theta) * np.sin(phi),
                                   np.cos(theta)])

        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(number_of_rays):
            ray_start_pos.append(position)

            ray_dir = rot_matrix@ray_directions[:, i]

            ray_end_pos.append(position-ray_length*ray_dir)

        results = p.rayTestBatch(ray_start_pos, ray_end_pos)
        return results
