import pybullet as p
import pybullet_industrial as pi
import numpy as np

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

def skew(v):
    """3-vector → 3×3 skew matrix."""
    x,y,z = v
    return np.array([[   0, -z,  y],
                     [   z,  0, -x],
                     [  -y,  x,  0]])

def transform_compliance(C_e: np.ndarray, r: list[float]) -> np.ndarray:
    """
    Adjoint wrench transform of compliance from link-origin (e) to point p = e + r.
    C_p = Ad * C_e * Ad^T
    """
    I3 = np.eye(3)
    r_skew = skew(r)
    Ad = np.block([
        [I3,         np.zeros((3,3))],
        [-r_skew,    I3]
    ])  # shape (6,6)
    return Ad @ C_e @ Ad.T

def compute_cartesian_compliance(
    body: int,
    linkIndex: int,
    localPosition: list[float],
    jointIndices: list[int],
    jointStiffness: list[float]
) -> np.ndarray:
    """
    Compute 6×6 Cartesian compliance at an arbitrary localPosition.
    If localPosition != [0,0,0], we transform the origin-compliance via the wrench adjoint.
    """
    # --- 1) get Jacobian at origin ---
    zero_vel = [0.0]*len(jointIndices)
    J_lin0, J_ang0 = p.calculateJacobian(
        body, linkIndex, [0,0,0],
        [p.getJointState(body,i)[0] for i in jointIndices],
        zero_vel, zero_vel
    )
    J0 = np.vstack((np.array(J_lin0), np.array(J_ang0)))  # 6×n

    # --- 2) build joint stiffness matrix and invert ---
    K = np.diag(jointStiffness)
    K_inv = np.linalg.inv(K)

    # --- 3) compliance at link origin ---
    C_e = J0 @ K_inv @ J0.T  # 6×6

    # --- 4) if offset, apply wrench adjoint transform ---
    if any(abs(x) > 1e-12 for x in localPosition):
        return transform_compliance(C_e, localPosition)
    else:
        return C_e


if __name__ == "__main__":
    import os

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
    
    def get_moving_joints(body):
        """
        Return a list of all joint indices in 'body' whose type is not FIXED.
        """
        moving = []
        for i in range(p.getNumJoints(body)):
            joint_info = p.getJointInfo(body, i)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                moving.append(i)
        return moving

    old_transform = None
    while True:
        p.stepSimulation()
        transform = compute_relative_transform(
            comau.urdf, comau._default_endeffector_id,
            kuka.urdf, kuka._default_endeffector_id)
        
        moving_joints = get_moving_joints(comau.urdf)
        comau_compliance = compute_cartesian_compliance(
            comau.urdf, comau._default_endeffector_id,
            [0, 0, 0],
            moving_joints,
            [1.0] * len(moving_joints)
        )
        
        if old_transform is None:
            old_transform = transform
        
        #check if position has changed
        if np.linalg.norm(np.array(transform[0])-np.array(old_transform[0])) > 0.1:
            print("Transform from kuka to comau:")
            print("Position: ", transform[0])
            print("Orientation: ", transform[1])
            print("Compliance: ", comau_compliance)

            old_transform = transform

