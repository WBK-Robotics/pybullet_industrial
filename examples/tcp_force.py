import os
import pybullet as p
import pybullet_industrial as pi
import pybullet_data


def spawn_pendulum(start_position):
    dirname = os.path.dirname(__file__)
    urdf_path = os.path.join(
        dirname, 'robot_descriptions', 'pendulum.urdf')
    pendulum = pi.EndeffectorTool(
        urdf_path, start_position, [0, 0, 0, 1])

    p.resetJointState(pendulum.urdf, 0, targetValue=0.5)
    return pendulum


if __name__ == "__main__":

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=15000)

    pendulum1 = spawn_pendulum([0, 0, 0])

    p.setTimeStep(0.5)
    steps = 10000
    while True:
        for _ in range(steps):
            pendulum1.apply_tcp_force([0, 0, 20.0], world_coordinates=True)
            p.stepSimulation()
            position, orientation = pendulum1.get_tool_pose()
        print(position)

        for _ in range(steps):
            pendulum1.apply_tcp_force(
                [0, 0, 0.0], world_coordinates=False)
            p.stepSimulation()
            pendulum_state = p.getLinkState(
                pendulum1.urdf, 0, computeLinkVelocity=1)
        print(pendulum_state[7])

        # for _ in range(steps):
        #    pendulum1.apply_tcp_force([0, 1, 0])
        #pendulum4.apply_tcp_torque([00, 2.1, 00])

        # print(pendulum_state[0])
        #position, orientation = pendulum1.get_tool_pose()
        # print(position)
        # p.stepSimulation()
