import os
import pybullet as p
import pybullet_industrial as pi


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_nj290_robot.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation, 'link6')
    robot = pi.RobotBase(urdf_file1, [1, 0, 0], start_orientation, 'link6')

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    drawing_position = [2.4, 0, 1.2]
    pi.draw_point(target_position, length=0.05)

    colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
    currentColor = 0

    p.getCameraImage(64, 64, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    while (1):
        objectUid, object_index = pi.get_object_id_from_mouse()
        if (objectUid >= 0):
            p.changeVisualShape(objectUid, object_index,
                                rgbaColor=colors[currentColor])
            currentColor += 1
            if (currentColor >= len(colors)):
                currentColor = 0
