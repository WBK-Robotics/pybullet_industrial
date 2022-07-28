import numpy as np
import pybullet as p
from pybullet_industrial.endeffector_tool import EndeffectorTool


class Camera(EndeffectorTool):
    def __init__(self, urdf_model: str, start_position, start_orientation, camera_parameters, coupled_robot=None):
        """AI is creating summary for __init__

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            camera_parameters (Dict[str,float]): A dictionary containing the camera parameters.
                                                 The parameters are: width, height, fov, aspect ratio, 
                                                 near plane distance and far plane distance.
            default_endeffector (str, optional): The default endeffector used 
                                                 when controlling the robots position
        """
        super().__init__(urdf_model, start_position, start_orientation, coupled_robot)

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

    def set_camera_parameters(self, camera_parameters):
        """Sets the camera parameters

        Args:
            camera_parameters (Dict[str,float]): A dictionary containing the camera parameters
        Raises:
            KeyError: If a key is not a valid parameter
        """
        for key in camera_parameters:
            if not key in self.camera_parameters:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.properties.keys()))
            self.camera_parameters[key] = camera_parameters[key]

        self.projection_matrix = self.__get_projection_matrix()

    def get_image(self):
        """Captures a camera image of the current simulation

        Returns:
            [type]: A list of pixel colors in rgb format in 255 color format
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
        _, _, img, _, _ = p.getCameraImage(
            width, height, view_matrix, self.projection_matrix)
        return img
