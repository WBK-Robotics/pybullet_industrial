import pybullet as p
import pybullet_industrial as pi


def spawn_voxel_block(base_position, dimensions, voxel_size):
    half_extents = voxel_size*0.5
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        rgbaColor=[1, 1, 1, 1],
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
                                 basePosition=base_position,
                                 batchPositions=batchPositions,
                                 useMaximalCoordinates=True)

    return bodyUids


p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=4,
                            minimumSolverIslandSize=1024)

# disable rendering during creation.
p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
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
    p.stepSimulation()
    objectUid, object_index = pi.get_object_id_from_mouse()
    if (objectUid >= 0):
        p.removeBody(objectUid)
