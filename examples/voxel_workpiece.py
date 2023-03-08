""" This example demonstrates the voxel engine of pybullet industrial.
    It creates multiple voxel blocks with different siced voxels and removes the using a remover 
    tool which accts as a water jet.
"""
import os

import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'maximum distance': 2.0,
                          'opening angle': 0.0,
                          'number of rays': 1}

    position = [0.01, -0.5, 1.2]
    remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)

    p.setPhysicsEngineParameter(numSolverIterations=4,
                                minimumSolverIslandSize=1024)

    p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
    # disable rendering during creation
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    size_progression = [1, 0.5, 0.2, 0.1, 0.05, 0.02]
    start = 0
    for size in size_progression:
        pi.spawn_material_block([0, start, 0],
                                [size, size, size],
                                pi.MetalVoxel,
                                {'particle size': size/10})
        start += size

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        remover.remove()
        remover.remove()
        remover.remove()
        remover.remove()
        position[1] += 0.005
        remover.set_tool_pose(position, [0, 0, 0, 1])
        p.stepSimulation()
        objectUid, object_index = pi.get_object_id_from_mouse()
        if (objectUid >= 0):
            p.removeBody(objectUid)
