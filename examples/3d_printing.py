import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np


def build_circular_path(center, radius, min_angle, max_angle, steps, height):
    """Function which builds a circular path

    Args:
        center (array): the center of the circle
        radius (float): the radius of the circle
        min_angle (float): minimum angle of the circle path
        max_angle (float): maximum angle of the circle path
        steps (int): the number of steps between min_angle and max_angle

    Returns:
        array: array of 3 dimensional path points
    """
    path = np.zeros((3, steps))
    path[2, :] = height
    for i in range(steps):
        path_angle = min_angle+i*(max_angle-min_angle)/steps
        new_position = center + radius * \
            np.array([np.sin(path_angle), np.cos(path_angle)])
        path[:2, i] = new_position
    return path


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    plastic = pi.Plastic(0.03, [1, 0, 0, 1])

    extruder_properties = {'maximum distance': 0.5,
                           'opening angle': 0,
                           'material': plastic,
                           'number of rays': 1}
    extruder = pi.Extruder(
        urdf_file2, [1.9, 0, 1.2], start_orientation, extruder_properties)
    extruder.couple(robot, 'link6')

    target_position = np.array([1.9, 0])
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    steps = 100
    base_height = 1.03
    test_path = build_circular_path(
        target_position, 0.3, 0, 2*np.pi, steps, base_height)
    for i in range(20):
        extruder.set_tool_pose(test_path[:, 0], target_orientation)
        for _ in range(50):
            p.stepSimulation()

    while True:
        test_path = build_circular_path(
            target_position, 0.3, 0, 2*np.pi, steps, base_height)
        pi.draw_path(test_path)
        base_height = base_height+0.03
        for i in range(steps):
            extruder.set_tool_pose(test_path[:, i], target_orientation)
            position, orientation = extruder.get_tool_pose()
            print(extruder.extrude())

            for _ in range(30):
                p.stepSimulation()
