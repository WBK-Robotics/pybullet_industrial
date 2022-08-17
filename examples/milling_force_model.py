import pybullet as p
import pybullet_industrial as pi
import os


def calculate_hit_percentage(remover):
    position, orientation = remover.get_tool_pose()
    raycast_results = remover.cast_rays(position, orientation)
    hit_positions = sum(
        [raycast_result[0] != -1 for raycast_result in raycast_results])
    nonhit_positions = sum(
        [raycast_result[0] == -1 for raycast_result in raycast_results])
    return hit_positions/(hit_positions+nonhit_positions)


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'maximum distance': 2.0,
                          'opening angle': 0.4,
                          'number of rays': 200}

    position = [0.00, 0.0, 0.6]
    corner_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    corner_remover.set_tool_pose(position, [0, 0, 0, 1])

    position = [0.00, 0.25, 0.6]
    edge_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    edge_remover.set_tool_pose(position, [0, 0, 0, 1])

    position = [0.25, 0.25, 0.6]
    full_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    full_remover.set_tool_pose(position, [0, 0, 0, 1])

    p.setPhysicsEngineParameter(numSolverIterations=4,
                                minimumSolverIslandSize=1024)

    p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
    # disable rendering during creation
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    size = 0.5
    pi.spawn_material_block([0, 0, 0],
                            [size, size, size],
                            pi.MetalVoxel,
                            {'particle size': size/10})

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        print("Corner: {}".format(calculate_hit_percentage(corner_remover)))
        print("Edge: {}".format(calculate_hit_percentage(edge_remover)))
        print("Full: {}".format(calculate_hit_percentage(full_remover)))

        p.stepSimulation()
        objectUid, object_index = pi.get_object_id_from_mouse()
        if (objectUid >= 0):
            p.removeBody(objectUid)
