import os
from copy import deepcopy
import unittest

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


dirname = os.path.dirname(__file__)
parentDir = os.path.dirname(dirname)
urdf_file1 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
urdf_file2 = os.path.join(parentDir, 'examples',
                          'robot_descriptions', 'milling_head.urdf')


def build_vertical_stack(start_position, extruder, step_num, step_size):
    target_position = deepcopy(start_position)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    for i in range(20):
        extruder.set_tool_pose(target_position, target_orientation)
        for _ in range(50):
            p.stepSimulation()

    relative_path = np.ones((3, step_num))*np.inf
    for i in range(step_num):
        target_position[2] = target_position[2]+step_size
        extruder.set_tool_pose(target_position, target_orientation)
        for _ in range(10):
            p.stepSimulation()
        position, _ = extruder.get_tool_pose()
        material_ids = extruder.extrude()
        if material_ids:
            material_position, _ = p.getBasePositionAndOrientation(
                material_ids[0])
            relative_path[:, i] = position-material_position

    return relative_path


class TestExtruder(unittest.TestCase):

    def test_extrusion_distance(self):
        """This tests checks wheter the extruder only extrudes material between [0,max distance]
        """
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        monastryId = p.createCollisionShape(p.GEOM_MESH,
                                            fileName="samurai_monastry.obj",
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
        p.createMultiBody(0, monastryId, baseOrientation=orn)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        plasstic_diameter = 0.03
        plastic = pi.Plastic(plasstic_diameter, [1, 0, 0, 1])

        extruder_properties = {'maximum distance': 0.5,
                               'opening angle': 0, 'material': plastic, 'number of rays': 1}
        extruder = pi.Extruder(
            urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
        extruder.couple(robot, 'link6')

        target_position = np.array([1.9, -0.3, 0])
        test_path_zero = build_vertical_stack(
            target_position, extruder, 20, plasstic_diameter/2)  # should be zero
        # discard first printed particles as they sit on the surface and not each other
        not_extruding_beyond_zero = (test_path_zero[:, 2:] < 10**-3).all()

        target_position = np.array([1.9, 0.3, 0.6])
        test_path_out_of_reach = build_vertical_stack(
            target_position, extruder, 20, plasstic_diameter/2)  # should be inf
        not_extruding_beyond_max = (test_path_out_of_reach == np.inf).all()

        p.disconnect()
        self.assertTrue(not_extruding_beyond_zero and not_extruding_beyond_max)

    def test_plastic_buildup(self):
        """This test checks that the plastic particles build up on top of each other
        """
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        monastryId = p.createCollisionShape(p.GEOM_MESH,
                                            fileName="samurai_monastry.obj",
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
        p.createMultiBody(0, monastryId, baseOrientation=orn)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        plasstic_diameter = 0.03
        plastic = pi.Plastic(plasstic_diameter, [1, 0, 0, 1])

        extruder_properties = {'maximum distance': 0.5,
                               'opening angle': 0, 'material': plastic, 'number of rays': 1}
        extruder = pi.Extruder(
            urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
        extruder.couple(robot, 'link6')

        target_position = np.array([1.9, 0, 0.2])
        # should steadily decrease with constant factor
        test_path_decreasing = build_vertical_stack(
            target_position, extruder, 10, plasstic_diameter/2)
        vertical_distance_covered = np.ediff1d(test_path_decreasing[2])
        distance_is_decreasing_linearly = (
            vertical_distance_covered-np.mean(vertical_distance_covered) <= 10**-3).all()

        p.disconnect()
        self.assertTrue(distance_is_decreasing_linearly)

    def test_material_distribution(self):
        """This test checks wheter the extruder produces a uniform material spray given the opening angle.
           Since this requires changing the extruder properties the function also validates the change_extruder_properties method.
        """
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=5000)
        monastryId = p.createCollisionShape(p.GEOM_MESH,
                                            fileName="samurai_monastry.obj",
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
        p.createMultiBody(0, monastryId, baseOrientation=orn)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

        plasstic_diameter = 0.03
        plastic = pi.Plastic(plasstic_diameter, [1, 0, 0, 1])

        extruder_properties = {'maximum distance': 0.5,
                               'opening angle': 0, 'material': plastic, 'number of rays': 1}
        extruder = pi.Extruder(
            urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
        extruder.couple(robot, 'link6')

        target_position = np.array([1.9, 0.3, 0.6])
        extruder.change_extruder_properties(
            {'maximum distance': 0.7, 'opening angle': np.pi/6, 'number of rays': 300})
        for _ in range(20):
            extruder.set_tool_pose(
                target_position, p.getQuaternionFromEuler([0, 0, 0]))
            for _ in range(50):
                p.stepSimulation()

        extrusion_center = np.array(
            [target_position[0], target_position[1], 0])
        cone_height = target_position[2]
        cone_radius = np.tan(np.pi/6/2)*cone_height
        id_list = extruder.extrude()
        extruder.change_extruder_properties(
            {'maximum distance': 0.5, 'opening angle': 0, 'number of rays': 1})

        distance_from_center_list = []
        for ids in id_list:
            material_position, _ = p.getBasePositionAndOrientation(ids)
            material_position = np.array(material_position)
            material_position[2] = 0
            distance_from_center = np.linalg.norm(
                material_position-extrusion_center)
            distance_from_center_list.append(distance_from_center)

        mean_is_uniform = (
            np.abs(np.mean(distance_from_center_list)-cone_radius/2) <= 10**-1)
        var_is_uniform = (np.var(distance_from_center_list) -
                          1/12*cone_radius**2 <= 10**-1)
        material_distributed_uniform = mean_is_uniform and var_is_uniform

        p.disconnect()
        self.assertTrue(material_distributed_uniform)


if __name__ == '__main__':
    p.connect(p.DIRECT)
    unittest.main()
    p.disconnect()
