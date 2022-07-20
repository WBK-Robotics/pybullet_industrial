import os
import pybullet as p
import pybullet_industrial as pi
import pybullet_data

if __name__ == "__main__":

    physics_client = p.connect(p.GUI)
    #p.setGravity(0, 0, -10)

    urdf_path = os.path.join(
        pybullet_data.getDataPath(), "cartpole.urdf")

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    pendulum = pi.EndeffectorTool(
        urdf_path, [0, 0, 0], start_orientation)

    p.createConstraint(pendulum.urdf,
                       -1, pendulum.urdf, 0,
                       p.JOINT_FIXED,
                       [0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 0],
                       None,
                       [0, 0, 0])

    p.resetJointState(pendulum.urdf, 1, targetValue=0.5)
    p.setTimeStep(1)
    pi.draw_robot_frames(pendulum, life_time=0)
    while True:
        pendulum.apply_tcp_force([-10, 0, 0],world_coordinates=True)
        #pendulum.apply_tcp_torque([00, 10, 00])
        position, orientation = pendulum.get_tool_pose()
        p.stepSimulation()
