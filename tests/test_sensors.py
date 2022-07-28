import os
import unittest

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


class TestCamera(unittest.TestCase):

    def test_camera_image(self):
        """This tests checks wheter the image has the correct dimension, displays the correct colors
           and if changing the camera parameters results in changed image dimensions.
        """
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file = os.path.join(
            parentDir, 'examples', 'robot_descriptions', 'camera.urdf')

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)

        start_orientation = p.getQuaternionFromEuler([0, np.pi/2, 0])
        camera_parameters = {'width': 480,
                             'height': 240,
                             'fov': 60,
                             'aspect ratio': 1,
                             'near plane distance': 0.01,
                             'far plane distance': 100}
        camera = pi.Camera(urdf_file, [-0.6, 0, 0],
                           start_orientation, camera_parameters)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        cube_id = p.loadURDF("cube.urdf", [0, 0, 0], useFixedBase=True)

        colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
        colors_255 = [[191, 0, 0, 255], [0, 191, 0, 255],
                      [0, 0, 191, 255], [191, 191, 191, 255]]
        image_correct = True
        for i in range(4):
            p.changeVisualShape(cube_id, -1, rgbaColor=colors[i])
            p.stepSimulation()
            img = camera.get_image()
            image_correct = image_correct and all(img[0][0] == colors_255[i])

        self.assertTrue(image_correct)

        image_shape = np.shape(img)
        dimensions_correct = image_shape[0] == camera_parameters[
            'height'] and image_shape[1] == camera_parameters['width']
        self.assertTrue(dimensions_correct)

        camera.set_camera_parameters({'height': 2})
        img = camera.get_image()
        image_shape = np.shape(img)
        dimensions_changed = image_shape[0] == 2
        self.assertTrue(dimensions_changed)
        p.disconnect()


if __name__ == '__main__':
    unittest.main()
