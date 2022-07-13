import pybullet as p
import pybullet_industrial as pi
import os



def spawn_voxel_block(base_position, dimensions, voxel_size, color=[1, 1, 1, 1]):
    half_extents = voxel_size*0.5
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        rgbaColor=color,
                                        halfExtents=[half_extents, half_extents, half_extents])
    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                              halfExtents=[half_extents, half_extents, half_extents])

    batchPositions = []

    for x in range(int(dimensions[0]/voxel_size)):
        for y in range(int(dimensions[1]/voxel_size)):
            for z in range(int(dimensions[2]/voxel_size)):
                batchPositions.append(
                    [x * voxel_size+base_position[0]+half_extents,
                     y * voxel_size+base_position[1]+half_extents,
                     z * voxel_size+base_position[2]+half_extents])

    bodyUids = p.createMultiBody(baseMass=0.0,
                                 baseInertialFramePosition=[0, 0, 0],
                                 baseCollisionShapeIndex=collisionShapeId,
                                 baseVisualShapeIndex=visualShapeId,
                                 batchPositions=batchPositions,
                                 useMaximalCoordinates=True)

    return bodyUids


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
        spawn_voxel_block([0, start, 0],
                          [size, size, size],
                          size/10)
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
