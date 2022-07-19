import os

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    extruder_properties = {'maximum distance': 0.5,
                           'opening angle': 0,
                           'material': pi.Plastic,
                           'material properties': {'particle size': 0.03},
                           'number of rays': 1}
    extruder = pi.Extruder(
        urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
    extruder.couple(robot, 'link6')

    remover_properties = {'maximum distance': 0.02,
                          'opening angle': 0,
                          'number of rays': 1}
    remover = pi.Remover(
        urdf_file2, [1.9, 1, 1.2], start_orientation, remover_properties)
    p.changeVisualShape(remover.urdf, -1, rgbaColor=[0, 0, 1, 1])

    # Defining a roundet rectangular path
    center_position = np.array([1.9, 0.0, 1.03])
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    steps = 20
    corner_point_00 = center_position+np.array([-0.2, -0.2, 0])
    corner_point_01 = center_position+np.array([-0.2, 0.2, 0])
    corner_point_10 = center_position+np.array([-0.1, 0.3, 0])
    corner_point_11 = center_position+np.array([0.1, 0.3, 0])
    corner_point_20 = center_position+np.array([0.2, 0.2, 0])
    corner_point_21 = center_position+np.array([0.2, -0.2, 0])
    corner_point_30 = center_position+np.array([0.1, -0.3, 0])
    corner_point_31 = center_position+np.array([-0.1, -0.3, 0])

    test_path = pi.linear_interpolation(corner_point_00, corner_point_01, 10)
    side_1 = pi.linear_interpolation(corner_point_10, corner_point_11, 10)
    side_2 = pi.linear_interpolation(corner_point_20, corner_point_21, 10)
    side_3 = pi.linear_interpolation(corner_point_30, corner_point_31, 10)

    corner_0 = pi.circular_interpolation(
        corner_point_01, corner_point_10, 0.1, 5)
    corner_1 = pi.circular_interpolation(
        corner_point_11, corner_point_20, 0.1, 5)
    corner_2 = pi.circular_interpolation(
        corner_point_21, corner_point_30, 0.1, 5)
    corner_3 = pi.circular_interpolation(
        corner_point_31, corner_point_00, 0.1, 5)

    test_path.append(corner_0)
    test_path.append(side_1)
    test_path.append(corner_1)
    test_path.append(side_2)
    test_path.append(corner_2)
    test_path.append(side_3)
    test_path.append(corner_3)

    test_path.draw()

    extruding = 1
    current_particles = []
    while True:
        for _ in range(20):
            extruder.set_tool_pose(
                test_path.positions[:, 0], target_orientation)
            for _ in range(50):
                p.stepSimulation()
        if extruding:
            for positions, orientations, _ in test_path:
                extruder.set_tool_pose(positions, orientations)
                particle = extruder.extrude()
                current_particles.append(particle[0].particle_id)

                for _ in range(30):
                    p.stepSimulation()
            extruder.decouple()
            remover.couple(robot, 'link6')
            extruding = 0
            continue
        if not extruding:
            for positions, orientations, _ in test_path:
                for _ in range(3):
                    removed_particles = remover.remove()
                    for elements in removed_particles:
                        current_particles.remove(elements)
                remover.set_tool_pose(positions, orientations)
                for _ in range(3):
                    removed_particles = remover.remove()
                    for elements in removed_particles:
                        current_particles.remove(elements)

                for _ in range(30):
                    p.stepSimulation()
            remover.decouple()
            extruder.couple(robot, 'link6')
            extruding = 1
