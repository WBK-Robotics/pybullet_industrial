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
def move_along_path(endeffektor: pi.EndeffectorTool, path, target_orientation, stop=False):
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
    urdf_file4 = os.path.join(dirname,
                              'robot_descriptions', 'screwDriver.urdf')
    urdf_file3 = os.path.join(dirname,
                              'robot_descriptions', 'cube_small.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=10000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    FoFaPath = os.path.join(dirname,
                            'Objects', 'FoFa', 'FoFa.urdf')
    p.loadURDF(FoFaPath, useFixedBase=True, globalScaling=0.001)


    p.setGravity(0, 0, -10)
    start_pos = np.array([2.0, -6.5, 0])
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, start_pos, start_orientation)

    start_pos2 = np.array([6.25, -6.5, 0])
    start_orientation2 = p.getQuaternionFromEuler([0, 0, np.pi])
    robot2 = pi.RobotBase(urdf_file1, start_pos2, start_orientation2)

    p.loadURDF("cube.urdf", np.array([2.125, 0, 0.5]) + start_pos, useFixedBase=True)


    gripper = pi.Gripper(
        urdf_file2, [1.9, 0, 1.2]+start_pos, start_orientation)
    gripper.couple(robot, 'link6')

    start_orientation_sg = p.getQuaternionFromEuler([0, 0, np.pi / 2])
    suctionGripper = pi.SuctionGripper(
        urdf_file4, [-1.9, 0, 5]+start_pos2, start_orientation, suction_links=["tcp"])

    suctionGripper.couple(robot2, 'link6')

    p.resetDebugVisualizerCamera(cameraDistance=4.8, cameraYaw=50.0, cameraPitch=-30,
                                 cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)




    start_orientation_gr = p.getQuaternionFromEuler([-np.pi, 0, 0])
    steps = 100
    base_height = 1.03
#    for c in gripper._gripper_constraints:
#        print(p.getConstraintInfo(c))
    safepoint1 = start_pos+[1.9, 0.5, 2.0]
    safepoint2 = start_pos2+[-1.9, -0.5, 2.0]
    grippoint1 = [1.9, -0.25, 1.05] + start_pos
    grippoint1_2 = grippoint1 + [0, 0, 0.1]
    droppoint1 = [-1.9, 0.25, 1.15] + start_pos2
    grippoint2 = droppoint1.copy() + [0, 0, -0.1105]
    grippoint2_2 = grippoint2 + [0, 0, 0.1]
    droppoint2 = grippoint1.copy() + [0, 0, 0.1]

    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)

    for _ in range(8):
        gripper.set_tool_pose(safepoint1, start_orientation_gr)
        gripper.actuate(0.0)
        suctionGripper.set_tool_pose(safepoint2, start_orientation_sg)
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)



    delta=0.1
    p.loadURDF(urdf_file3, grippoint1, useFixedBase=False)
    while True:
        path = build_linear_path(np.array(safepoint1), np.array(grippoint1_2), 10)
        move_along_path(gripper, path,start_orientation_gr)
        path = build_linear_path(np.array(grippoint1_2), np.array(grippoint1), 10)
        move_along_path(gripper, path,start_orientation_gr)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        gripper.actuate(1.0)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        path = build_linear_path(np.array(grippoint1), np.array(grippoint1_2), 10)
        move_along_path(gripper, path, start_orientation_gr)
        path = build_linear_path(np.array(grippoint1_2), np.array(droppoint1), 10)
        move_along_path(gripper, path,start_orientation_gr)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        gripper.actuate(0.0)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        path = build_linear_path(np.array(droppoint1), np.array(safepoint1), 10)
        move_along_path(gripper, path,start_orientation_gr)

        path = build_linear_path(np.array(safepoint2), np.array(grippoint2_2), 10)
        move_along_path(suctionGripper, path,start_orientation_sg)
        path = build_linear_path(np.array(grippoint2_2), np.array(grippoint2), 10)
        move_along_path(suctionGripper, path,start_orientation_sg)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        suctionGripper.activate(0.001)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        path = build_linear_path(np.array(grippoint2), np.array(grippoint2_2), 10)
        move_along_path(suctionGripper, path,start_orientation_sg)
        path = build_linear_path(np.array(grippoint2_2), np.array(droppoint2), 10)
        move_along_path(suctionGripper, path,start_orientation_sg)
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        suctionGripper.deactivate()
        for _ in range(25):
            p.stepSimulation()
            time.sleep(0.01)
        path = build_linear_path(np.array(droppoint2), np.array(safepoint2), 10)
        move_along_path(suctionGripper, path,start_orientation_sg)