""" Simple Example showing how force can be applied to the end effector.
    This example uses the urdf of a simple pendulum as an end effector 
    where the tip of the pendulum is the tool center point.
    
    Different forces are applied to the pendulum in different ways:
    1. Apply a force in world coordinates. 
       This causes the pendulum to rest along the axis of the force
    2. Apply a force without specifying the coordinate system.
       This results in the same behavior as 1. since the default is world coordinates.
    3. Apply a force in tool coordinates.
       This causes the pendulum to rotate around its axis, since the force vector changes direction
       with the orientation of the pendulum.
    4. Apply a torque in world coordinates.
       This causes the pendulum to rotate around its axis, like the force in tool coordinates.
"""
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
    pendulum2 = spawn_pendulum([0, 0.6, 0])
    pendulum3 = spawn_pendulum([0, 1.2, 0])
    pendulum4 = spawn_pendulum([0, 1.8, 0])

    p.setTimeStep(0.5)
    steps = 10000
    while True:
        for _ in range(steps):
            pendulum1.apply_tcp_force([-10, 0, 20.0], world_coordinates=True)
            pendulum2.apply_tcp_force([0, 1, 0])
            pendulum3.apply_tcp_force(
                [-1, 0, 0.0], world_coordinates=False)
            pendulum4.apply_tcp_torque([00, 1, 00])

            p.stepSimulation()

            position1, _ = pendulum1.get_tool_pose()
            position2, _ = pendulum2.get_tool_pose()
            pendulum_state3 = p.getLinkState(
                pendulum3.urdf, 0, computeLinkVelocity=1)
            pendulum_state4 = p.getLinkState(
                pendulum4.urdf, 0, computeLinkVelocity=1)
        print(position1, position2, pendulum_state3[7], pendulum_state4[7])
