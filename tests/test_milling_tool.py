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

    def test_force_direction(self):
        """This test checks wheter the tool force is provided into
           the correct direction by cutting agains a wall."""
        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=5000)

        remover_properties = {'diameter': 0.1,
                              'rotation speed': 2*np.pi/5,
                              'number of teeth': 5,
                              'height': 0.1,
                              'number of rays': 10}

        milling_tool = pi.MillingTool(
            urdf_file, [0, 0, 0], [0, 0, 0, 1], remover_properties)
        milling_tool.set_tool_pose([0, 0, 0], [0, 0, 0, 1])

        pi.MetalVoxel([0, 0, 0, np.array([0.0, 0.05, 0])],
                      {'particle size': 0.1, 'color': [1, 0, 0, 1]})

        for _ in range(100):
            p.stepSimulation()
        tool_position, tool_orientation = milling_tool.get_tool_pose()

        force_x = []
        force_y = []
        for _ in range(50):
            ray_cast_results = milling_tool.cast_rays(
                tool_position, tool_orientation)
            _, cutting_depth = milling_tool.get_cutting_state(
                ray_cast_results)

            teeth_angles = [i * 2*np.pi/milling_tool.properties['number of teeth'] +
                            milling_tool.current_angle
                            for i in range(milling_tool.properties['number of teeth'])]

            cutting_force = milling_tool.force_model(0.01,
                                                     cutting_depth,
                                                     teeth_angles)
            # print(cutting_force)
            ray_cast_results = milling_tool.cast_rays(
                tool_position, tool_orientation)

            milling_tool.current_angle += np.pi*2/5*0.02

            force_x.append(cutting_force[0])
            force_y.append(cutting_force[1])
            p.stepSimulation()

        p.disconnect()

        self.assertAlmostEqual(np.mean(force_x), 0.0, places=2)
        self.assertTrue(np.abs(np.mean(force_y)) >=
                        np.abs(np.mean(force_x))*10)
        self.assertTrue(np.mean(force_y) < 0)


if __name__ == '__main__':
    unittest.main()
