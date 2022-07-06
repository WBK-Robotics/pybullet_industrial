import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np
import time




def build_circular_path(center, radius, min_angle, max_angle, steps,height):
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
    path[2,:]=height
    for i in range(steps):
        path_angle = min_angle+i*(max_angle-min_angle)/steps
        new_position = center + radius * \
            np.array([np.sin(path_angle), np.cos(path_angle)])
        path[:2, i] = new_position
    return path
def build_linear_path(start, end, steps):
    """Function which builds a linar path
    Args:
        start (array): start point
        end (array): end point
        steps (int): the number of steps between start and end point
    Returns:
        array: array of 3 dimensional path points
    """
    path = np.zeros((steps,3))
    for i in range(steps):
        path[i] = start+(end-start)*(i/(steps-1))
    return path
def move_along_path(endeffektor: pi.EndeffectorTool, path, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (endeffector_tool): Endeffector to be moved.
        path (array): Array of points defining the path.

    """
    for pnt in path:
        endeffektor.set_tool_pose(pnt, target_orientation)
        for _ in range(10):
            p.stepSimulation()
    if stop:
        for _ in range(100):
            p.stepSimulation()


if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m_clean.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'gripper_cad.urdf')
    urdf_file3 = os.path.join(dirname,
                              'robot_descriptions', 'cube_small.urdf')


    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=10000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    p.setGravity(0, 0, -10)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)


    gripper = pi.Gripper(
        urdf_file2, [1.9, 0, 1.2], start_orientation)
    gripper.couple(robot, 'link6')

    target_position = np.array([1.9, 0])
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    steps = 100
    base_height = 1.03
    test_path = build_circular_path(target_position, 0.3, 0, 2 * np.pi, steps, base_height)
    for c in gripper._gripper_constraints:
        print(p.getConstraintInfo(c))
    for _ in range(20):
        gripper.set_tool_pose([1.9, 0, 2.0], target_orientation)
        gripper.actuate(0.0)
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)
    spawnpoint=[1.9, 0.25, 1.05]
    grippoint=[1.9, 0.25, 1.05]
    droppoint1=[1.9, -0.25, 1.14]
    droppoint2=[1.9, -0.05, 1.14]
    delta=0.1
    p.loadURDF(urdf_file3, spawnpoint, useFixedBase=False)
    gripper.set_tool_pose([1.9, 0, 1.5], target_orientation)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)
    path = build_linear_path(np.array([1.9, 0, 1.5]), np.array(grippoint), 10)
    while True:
        move_along_path(gripper, path, True)
        gripper.actuate(1.0)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        path = build_linear_path(np.array(grippoint), np.array(droppoint2), 10)
        move_along_path(gripper, path)
        path = build_linear_path(np.array(droppoint2), np.array(droppoint1), 10)
        move_along_path(gripper, path, True)
        gripper.actuate(0.0)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01)
        p.loadURDF(urdf_file3, spawnpoint, useFixedBase=False)
        path = build_linear_path(np.array(droppoint1), np.array(droppoint2), 10)
        move_along_path(gripper, path)

        path = build_linear_path(np.array(droppoint2), np.array(grippoint), 10)
        droppoint1[2]=droppoint1[2]+delta
        droppoint2[2]=droppoint2[2]+delta