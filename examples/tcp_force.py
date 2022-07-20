import os
import pybullet as p
import pybullet_industrial as pi
import pybullet_data

if __name__ == "__main__":

    physics_client = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    urdf_path = os.path.join(
        pybullet_data.getDataPath(), "cartpole.urdf")

    p.setPhysicsEngineParameter(numSolverIterations=5000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    pendulum = pi.EndeffectorTool(
        urdf_path, [1.9, 0, 1.2], start_orientation)

    p.setTimeStep(0.1)
    while True:
        pendulum.apply_tcp_force([0, 0, -10])
        p.stepSimulation()
