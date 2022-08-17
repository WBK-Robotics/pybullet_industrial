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

    for _ in range(100):
        p.stepSimulation()

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
    corner_average = 0
    edge_average = 0
    full_average = 0
    n = 1
    while (1):
        corner_average = corner_average + \
            (calculate_hit_percentage(corner_remover)-corner_average)/n
        edge_average = edge_average + \
            (calculate_hit_percentage(edge_remover)-edge_average)/n
        full_average = full_average + \
            (calculate_hit_percentage(full_remover)-full_average)/n
        print("Corner: {}".format(corner_average))
        print("Edge: {}".format(edge_average))
        print("Full: {}".format(full_average))

        p.stepSimulation()
        n += 1
