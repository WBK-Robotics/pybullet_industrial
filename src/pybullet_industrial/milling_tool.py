from typing import Dict

import numpy as np
import pybullet as p


from pybullet_industrial.endeffector_tool import EndeffectorTool
from pybullet_industrial.robot_base import RobotBase


class MillingTool(EndeffectorTool):
    """A class implementing a milling tool.
    The tool is modeled as a set of teeth that remove material
    and apply forces on the tcp frame according to the kienzle model.

    Args:
        urdf_model (str): The path to the urdf model of the milling tool
        start_position (np.array): The position of the tool center point
        start_orientation (np.array): The orientation of the tool center point
        milling_properties (Dict): A dictionary containing the properties of the milling tool.
                                   Default values are:
                                   'diameter':0.05, 'height':0.01,
                                   'number of rays':10,'rotation speed':0.1, 'number of teeth':5
                                   'material_specific_force':2500,'chip_thickness_exponent':0.26
                                   The parameters kc11 and m_c are specific for the material.
        coupled_robot (RobotBase, optional): A pybullet_industrial.RobotBase object if
                                             the robot is coupled from the start.
                                             Defaults to None.
        tcp_frame (str, optional): The name of the urdf_link
                                   describing the tool center point.
                                   Defaults to None in which case the last link is used.
        connector_frame (str, optional): The name of the urdf_link
                                         at which a robot connects.
                                         Defaults to None in which case the base link is used


    """

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 milling_properties: Dict, coupled_robot: RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.properties = {'diameter': 0.05,
                           'rotation speed': 1,
                           'number of teeth': 5,
                           'height': 0.1,
                           'number of rays': 10,
                           'material_specific_force': 2500,
                           'chip_thickness_exponent': 0.26}
        self.current_angle = 0
        if milling_properties is not None:
            self.change_properties(milling_properties)

    def get_cutting_state(self, ray_cast_result, tcp_frame=None):
        """A helpfer function calculating the cutting depth and speed of the tool.

        Args:
            ray_cast_result (list): The result of a batch ray cast as performed
                                      by the mill function
            tcp_frame (str, optional): The tool center point frame name. Defaults to None.

        Returns:
            float: the speed at which the cutting tool is moved into the material
            (np.array): an array of the depth of the cutting tool at each tooth
        """
        if tcp_frame is None:
            tcp_id = self._tcp_id
        else:
            tcp_id = self._convert_link_to_id(tcp_frame)

        link_state = p.getLinkState(self.urdf, tcp_id, computeLinkVelocity=1)
        cutting_speed = np.linalg.norm(link_state[7])

        ray_cast_per_teeth = [ray_cast_result[i:i+self.properties['number of rays']]
                              for i in range(0, len(ray_cast_result),
                                             self.properties['number of rays'])]

        # calculate the cutting depth based on the amount of rays that are hitting a body
        # if the cutting depth is zero, the tool is not cutting
        cutting_depth = [sum([ray_cast[0] != -1 for ray_cast in ray_cast_per_teeth[i]])
                         for i in range(self.properties['number of teeth'])]
        cutting_depth = np.array(
            cutting_depth)*self.properties['height']/self.properties['number of rays']

        return cutting_speed, cutting_depth

    def cast_rays(self, position: np.array, orientation: np.array, debug=False):
        """Randomly casts rays withn a given range from a specified position and orientation

        Args:
            position (np.array): start position of the raycast
            orientation (np.array): start orientation of the raycast
            debug (bool, optional): If true, the debug lines are drawn. Defaults to False.

        Returns:
            List: The result of a raycast
        """
        ray_start_pos = []
        ray_end_pos = []
        angle_between_teeth = 2*np.pi/self.properties['number of teeth']

        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(self.properties['number of teeth']):
            end_point = self.properties['diameter']*np.array(
                [np.sin(i*angle_between_teeth+self.current_angle),
                 np.cos(i*angle_between_teeth+self.current_angle),
                 0])
            for j in range(self.properties['number of rays']):
                height_adjustment = np.array(
                    [0, 0, j*self.properties['height']/self.properties['number of rays']])

                start_position = position+rot_matrix@height_adjustment
                end_position = position + \
                    rot_matrix@(end_point+height_adjustment)

                if debug:
                    p.addUserDebugLine(start_position, end_position,
                                       [1, 0, 0], 1, lifeTime=1)
                ray_start_pos.append(start_position)
                ray_end_pos.append(end_position)

        ray_cast_results = p.rayTestBatch(ray_start_pos, ray_end_pos)

        return ray_cast_results

    def calculate_process_force(self, ray_cast_results, tcp_frame=None):
        """Function calculating the process force of the milling tool.

        Args:
            ray_cast_results (list): The result of a batch ray cast
                                       as performed by the mill function
            tcp_frame (str): The name of the tool center point frame. Defaults to None.

        Returns:
            np.array: a 3 dimensional array containing the process force in the x,y,z direction
        """
        cutting_speed, cutting_depth = self.get_cutting_state(
            ray_cast_results, tcp_frame)
        teeth_angles = [i * 2*np.pi/self.properties['number of teeth'] +
                        self.current_angle for i in range(self.properties['number of teeth'])]
        cutting_force = self.force_model(cutting_speed,
                                         cutting_depth,
                                         teeth_angles)
        return cutting_force

    def mill(self, tcp_frame=None, debug=False):
        """Function that performs a milling operation.

        Args:
            tcp_frame (str, optional): The name of the tool center point frame. Defaults to None.
            debug (bool, optional): If true, the debug lines are drawn. Defaults to False.

        Returns:
            list: A list containing the ids of the bodies that were removed
        """

        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation, debug)

        cutting_force = self.calculate_process_force(
            ray_cast_results, tcp_frame)
        self.apply_tcp_force(cutting_force, tcp_frame)

        self.current_angle = self.current_angle + \
            self.properties['rotation speed']

        removed_objects = []
        for ray_intersection in ray_cast_results:
            if ray_intersection[0] != -1:
                p.removeBody(ray_intersection[0])
                removed_objects.append(ray_intersection[0])
        return removed_objects

    def force_model(self, cutting_speed, cutting_depth, teeth_angles):
        """A force model that is used to calculate the force that is applied to the tool.
        Args:
            cutting_speed (float): the speed at which the cutting tool is moved into the material
            cutting_depth (np.array): an array of the depth of the cutting tool at each tooth
            teeth_angles (list): the angles of the teeth on the tool
        Returns:
            np.array: an array of the force that is applied to the cutting tool at the tcp
        """
        h = cutting_speed / \
            (self.properties['rotation speed'] *
             self.properties['number of teeth'])
        k_c = self.properties['material_specific_force'] / \
            (h ** self.properties['chip_thickness_exponent'])

        force = np.zeros(3)

        for index, depth in enumerate(cutting_depth):
            cutting_force = k_c * depth * h
            force -= np.array([cutting_force*np.sin(teeth_angles[index]),
                               cutting_force*np.cos(teeth_angles[index]), 0])

        return force

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
