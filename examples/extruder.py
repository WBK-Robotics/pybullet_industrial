import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np

class GlueParticle:

    def __init__(self,particle_size,position) -> None:
        pass


def sample_spherical(npoints, opening_angle):
    phi = np.random.uniform(-np.pi,np.pi,npoints)
    theta = np.random.uniform(-0.5*opening_angle,0.5*opening_angle,npoints)
    x = np.sin(theta)* np.cos(phi)
    y = np.sin(theta)* np.sin(phi)
    z = np.cos(theta)
    vec = np.array([x,y,z])
    return vec


def cast_rays(position,orientation,opening_angle,numRays,rayLen):
    rayFrom = []
    rayTo = []
    rayIds = []

    rayMissColor = [0, 1, 0]

    replaceLines = True

    ray_directions = sample_spherical(numRays,opening_angle)
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    for i in range(numRays):
        rayFrom.append(position)

        ray_dir = rot_matrix@ray_directions[:,i]

        rayTo.append(position-rayLen*ray_dir)
        if (replaceLines):
            rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
        else:
            rayIds.append(-1)
    results = p.rayTestBatch(rayFrom, rayTo)
    return results

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    extruder = pi.EndeffectorTool(
        urdf_file2, [1.9, 0, 1.2], start_orientation)
    p.setRealTimeSimulation(1)
    extruder.couple(robot, 'link6')

    target_position = np.array([1.9, 0, 1.2])
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])

    while True:
        ray_pos,ray_ori = extruder.get_tool_pose()
        cast_rays(ray_pos,ray_ori,np.pi/3,1024,1)
        time.sleep(1)
            

