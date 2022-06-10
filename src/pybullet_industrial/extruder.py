from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool


class Extruder(EndeffectorTool):

    def __init__(self, urdf_model: str, start_position, start_orientation, extruder_properties: Dict, coupled_robots=None, tcp_frame=None, connector_frames=None):
        """Special Endeffector Tool which can extrude material from its tcp.

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            extruder_properties(Dict): A dictionary containing the properties of the extrusion head.
                                       During initialization only 'material' has to be set.
                                       Default Values are:
                                       'opening angle':0,'number of rays':1,
                                       'material properties': {'particle size':0.03,
                                                               'color' : [1, 0, 0, 1]},
                                       'maximum distance':1,'material':None,
            coupled_robot ([type], optional): A wbk_sim.Robot object if
                                              the robot is coupled from the start.
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link
                                          describing the tool center point.
                                          Defaults to None in which case the last link is used.
            connector_frame ([type], optional): The name of the urdf_link
                                                at which a robot connects.
                                                Defaults to None in which case the base link is used.

        Raises:
            ValueError: If no material is provided during initialization.
        """
        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robots, tcp_frame, connector_frames)

        self.extruder_properties = {'opening angle': 0,
                                    'number of rays': 1,
                                    'maximum distance': 1,
                                    'material': None,
                                    'material properties': {'particle size': 0.03,
                                                            'color': [1, 0, 0, 1]}}
        self.change_extruder_properties(extruder_properties)
        if self.extruder_properties['material'] is None:
            raise ValueError("The extruder requires a initial Material")

    def extrude(self, tcp_frame=None):
        """Extrudes material from the specified tcp_frame.

        Args:
            tcp_frame (str, optional): the name of the link from which to extrude the material.
                                       Defaults to None in which case the default tcp is used
        """
        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation,
                                          self.extruder_properties['opening angle'],
                                          self.extruder_properties['number of rays'],
                                          self.extruder_properties['maximum distance'])

        particle_list = []
        for i in range(self.extruder_properties['number of rays']):
            ray_intersection = ray_cast_results[i]
            if ray_intersection[0] != -1:
                particle = self.extruder_properties['material'](ray_intersection,
                                                                self.extruder_properties['material properties'])
                particle_list.append(particle)
        return particle_list

    def change_extruder_properties(self, new_properties: Dict):
        """Allows retroactive changes to the extruder properties.


        Args:
            new_properties (Dict): A dictionary containing values and keys that
                                   should be changed

        Raises:
            KeyError: If a key is not a valid extruder property
        """
        for key in new_properties:
            if not key in self.extruder_properties:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.extruder_properties.keys()))
            else:
                self.extruder_properties[key] = new_properties[key]

    @staticmethod
    def cast_rays(position, orientation, opening_angle, number_of_rays, ray_length):
        """Randomly casts rays withn a given range from a specified position and orientation

        Args:
            position ([type]): start position of the rays
            orientation ([type]): start orientation of the rays
            opening_angle ([type]): the opening angle in which the rays should be cast
            number_of_rays ([type]): The number of rays which are cast
            ray_length ([type]): The maximum range of the rays

        Returns:
            [type]: [description]
        """
        ray_start_pos = []
        ray_end_pos = []

        phi = np.random.uniform(-np.pi, np.pi, number_of_rays)
        theta = np.random.uniform(-0.5*opening_angle,
                                  0.5*opening_angle, number_of_rays)
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)
        ray_directions = np.array([x, y, z])

        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(number_of_rays):
            ray_start_pos.append(position)

            ray_dir = rot_matrix@ray_directions[:, i]

            ray_end_pos.append(position-ray_length*ray_dir)

        results = p.rayTestBatch(ray_start_pos, ray_end_pos)
        return results
