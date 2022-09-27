from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool
from pybullet_industrial.robot_base import RobotBase


class Camera(EndeffectorTool):
    """Special Endeffector Tool which can cast rays. Base for Extruder and Remover classes.

    Args:
        urdf_model (str): A valid path to a urdf file describint the tool geometry
        start_position (np.array): the position at which the tool should be spawned
        start_orientation (np.array): the orientation at which the tool should be spawned
        camera_parameters (Dict[str,float]): A dictionary containing the camera parameters.
                                             Default Values are:
                                             'width': 480,
                                             'height': 240,
                                             'fov': 60,
                                             'aspect ratio': 1,
                                             'near plane distance': 0.01,
                                             'far plane distance': 100
        coupled_robot (RobotBase, optional): A pybullet_industrial.RobotBase object if
                                             the robot is coupled from the start.
                                             Defaults to None.
        camera_frame (str, optional): The name of the urdf_link
                                      at which the camera is located.
                                      Defaults to None in which case the last link is used.
        connector_frame (str, optional): The name of the urdf_link
                                         at which a robot connects.
                                         Defaults to None in which case the base link is used.
    """

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 camera_parameters: Dict, coupled_robot: RobotBase = None,
                 camera_frame: str = None, connector_frame: str = None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, camera_frame, connector_frame)

        self.camera_parameters = {'width': 480,
                                  'height': 240,
                                  'fov': 60,
                                  'aspect ratio': 1,
                                  'near plane distance': 0.01,
                                  'far plane distance': 100}
        self.set_camera_parameters(camera_parameters)

        self.projection_matrix = self.__get_projection_matrix()

    def __get_projection_matrix(self):
        fov = self.camera_parameters['fov']
        aspect_ratio = self.camera_parameters['aspect ratio']
        near_plane = self.camera_parameters['near plane distance']
        far_plane = self.camera_parameters['far plane distance']
        projection_matrix = p.computeProjectionMatrixFOV(
            fov, aspect_ratio, near_plane, far_plane)
        return projection_matrix

    def set_camera_parameters(self, camera_parameters: Dict):
        """Sets the camera parameters

        Args:
            camera_parameters (Dict[str,float]): A dictionary containing the camera parameters
        Raises:
            KeyError: If a key is not a valid parameter
        """
        for key in camera_parameters:
            if not key in self.camera_parameters:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.camera_parameters))
            self.camera_parameters[key] = camera_parameters[key]

        self.projection_matrix = self.__get_projection_matrix()

    def get_image(self):
        """Captures a camera image of the current simulation

        Returns:
            np.array: A array of pixel colors in rgba format in 255 color format
        """
        link_state = p.getLinkState(
            self.urdf, self._tcp_id, computeForwardKinematics=True)
        com_p = np.array(link_state[0])
        com_o = np.array(link_state[1])
        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (0, 0, 1)  # z-axis
        init_up_vector = (0, 1, 0)  # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(
            com_p, com_p + 0.1 * camera_vector, up_vector)

        width = self.camera_parameters['width']
        height = self.camera_parameters['height']
        images = p.getCameraImage(
            width, height, view_matrix, self.projection_matrix)
        img = np.reshape(images[2], (height, width, 4))
        return img
