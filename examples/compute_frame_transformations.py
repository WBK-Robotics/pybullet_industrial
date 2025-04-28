import pybullet as p
import pybullet_industrial as pi

def get_frame_transform(body, linkIndex):
    """
    Returns (pos, orn) of the given body+linkIndex in world coordinates.
    linkIndex = -1 is treated as the base.
    """
    if linkIndex == -1:
        return p.getBasePositionAndOrientation(body)
    else:
        st = p.getLinkState(body, linkIndex, computeForwardKinematics=True)
        return st[4], st[5]  # worldLinkFramePosition, worldLinkFrameOrientation

def compute_relative_transform(
    bodyA, linkIndexA, 
    bodyB, linkIndexB
):
    """
    Compute the pose of (bodyB, linkIndexB) relative to (bodyA, linkIndexA).

    Returns:
        pos_B_in_A: [x,y,z]
        orn_B_in_A: quaternion [x,y,z,w]
    """
    # 1) get world poses
    posA, ornA = get_frame_transform(bodyA, linkIndexA)
    posB, ornB = get_frame_transform(bodyB, linkIndexB)

    # 2) invert A’s world transform → world→A
    invPosA, invOrnA = p.invertTransform(posA, ornA)

    # 3) apply to B’s world pose → A→B
    pos_B_in_A, orn_B_in_A = p.multiplyTransforms(
        invPosA, invOrnA,
        posB, ornB
    )
    return pos_B_in_A, orn_B_in_A

if __name__ == "__main__":
    import os
    import numpy as np

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file2 = os.path.join(
        dirname, 'robot_descriptions', 'kuka_robot.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    comau = pi.RobotBase(urdf_file1, [-2, 0, 0], start_orientation)
    start_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
    kuka = pi.RobotBase(urdf_file2, [2, 0, 0], start_orientation)
    
    old_transform = None
    while True:
        p.stepSimulation()
        transform = compute_relative_transform(
            comau.urdf, comau._default_endeffector_id,
            kuka.urdf, kuka._default_endeffector_id)
        
        if old_transform is None:
            old_transform = transform
        
        #check if position has changed
        if np.linalg.norm(np.array(transform[0])-np.array(old_transform[0])) > 0.1:
            print("Transform from kuka to comau:")
            print("Position: ", transform[0])
            print("Orientation: ", transform[1])

            old_transform = transform

