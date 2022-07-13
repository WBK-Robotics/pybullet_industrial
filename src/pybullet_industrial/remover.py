from typing import Dict

import pybullet as p

from pybullet_industrial.material import Plastic
from pybullet_industrial.raycaster import RayCaster


class Remover(RayCaster):

    def __init__(self, urdf_model: str, start_position, start_orientation, remover_properties: Dict,
                 coupled_robots=None, tcp_frame=None, connector_frames=None):
        """Special Remover Tool which can remove objects from the simulation.

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            remover_properties(Dict): A dictionary containing the properties of the extrusion head.
                                       During initialization only 'material' has to be set.
                                       Default Values are:
                                       'opening angle':0,'number of rays':1,
                                       'maximum distance':1
            coupled_robot ([type], optional): A wbk_sim.Robot object if
                                              the robot is coupled from the start.
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link
                                          describing the tool center point.
                                          Defaults to None in which case the last link is used.
            connector_frame ([type], optional): The name of the urdf_link
                                                at which a robot connects.
                                                Defaults to None in which case the base link is used.
        """

        super().__init__(urdf_model, start_position, start_orientation, remover_properties,
                         coupled_robots, tcp_frame, connector_frames)

        self.properties
        self.properties['material'] = Plastic
        self.properties['material properties'] = {
            'particle size': 0.03, 'color': [1, 0, 0, 1]}

        self.change_properties(remover_properties)
        if self.properties['material'] is None:
            raise ValueError("The extruder requires a initial Material")

    def remove(self, tcp_frame=None):
        """Removes simulation elements from the simulation environment.

        Args:
            tcp_frame (str, optional): the name of the link from which to extrude the material.
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
