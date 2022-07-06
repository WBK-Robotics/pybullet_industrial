import pybullet as p
import pybullet_industrial as pi
cid = p.connect(p.GUI)

p.setPhysicsEngineParameter(numSolverIterations=4,
                            minimumSolverIslandSize=1024)
p.setTimeStep(1. / 120.)

# disable rendering during creation.
p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


voxel_scale = [0.1, 0.1, 0.1]
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    halfExtents=voxel_scale)
collisionShapeId = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=voxel_scale
)  # MESH, vertices=vertices, collisionFramePosition=shift,meshScale=meshScale)


batchPositions = []

for x in range(32):
    for y in range(32):
        for z in range(10):
            batchPositions.append(
                [x * voxel_scale[0] * 2, y * voxel_scale[1] * 2, (0.0 + z) * voxel_scale[2] * 2])

bodyUids = p.createMultiBody(baseMass=0.0,
                             baseInertialFramePosition=[0, 0, 0],
                             baseCollisionShapeIndex=collisionShapeId,
                             baseVisualShapeIndex=visualShapeId,
                             basePosition=[0, 0, 2],
                             batchPositions=batchPositions,
                             useMaximalCoordinates=True)
# p.changeVisualShape(bodyUids[0], -1, textureUniqueId=texUid)

# p.syncBodyInfo()
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


while (1):
    p.stepSimulation()
    objectUid, object_index = pi.get_object_id_from_mouse()
    if (objectUid >= 0):
        p.removeBody(objectUid)
