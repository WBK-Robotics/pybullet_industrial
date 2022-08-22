import os
import unittest
import numpy as np
import pybullet as p

import pybullet_industrial as pi


dirname = os.path.dirname(__file__)
parentDir = os.path.dirname(dirname)
urdf_file = os.path.join(parentDir, 'examples',
                         'robot_descriptions', 'milling_head.urdf')


class TestMillingTool(unittest.TestCase):

    def test_cutting_depth(self):
        """This test checks wheter the cutting depth is correctly measured by spawning two particles
           of known diameter in fron of the teeth.
        """
        p.connect(p.DIRECT)

        milling_properties = {'diameter': 0.1,
                              'rotation speed': 2*np.pi/5,
                              'number of teeth': 5,
                              'height': 0.1,
                              'number of rays': 10}

        position = [0.00, 0.0, 0.6]
        milling_tool = pi.MillingTool(
            urdf_file, position, [0, 0, 0, 1], milling_properties)
        milling_tool.set_tool_pose(position, [0, 0, 0, 1])

        for _ in range(100):
            p.stepSimulation()

        pi.Plastic([0, 0, 0, position+np.array([0., 0.05, 0])],
                   {'particle size': 0.015, 'color': [1, 0, 0, 1]})
        pi.Plastic([0, 0, 0, position+np.array([0.05, 0., 0])],
                   {'particle size': 0.03, 'color': [1, 0, 0, 1]})
        tool_position, tool_orientation = milling_tool.get_tool_pose()
        ray_cast_results = milling_tool.cast_rays(
            tool_position, tool_orientation)
        _, cutting_depth = milling_tool.get_cutting_state(
            ray_cast_results)

        p.disconnect()

        self.assertEqual(cutting_depth[0], 0.02)
        # almost equal is needet due to python rounding errors for thirds.
        self.assertAlmostEqual(cutting_depth[1], 0.03, places=8)
        self.assertEqual(cutting_depth[2], 0)
        self.assertEqual(cutting_depth[3], 0)
        self.assertEqual(cutting_depth[4], 0)


if __name__ == '__main__':
    unittest.main()
