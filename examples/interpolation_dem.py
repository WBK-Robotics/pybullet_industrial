import os
import pybullet as p
import numpy as np
import pybullet_data
import pybullet_industrial as pi
from interpolation import linear_interpolation
from interpolation import circular_interpolation
from interpolation import spline_interpolation
from pybullet_industrial import ToolPath


def run_elementary_operations(elementary_operations: list):
    for operation in elementary_operations:
        operation()
        for _ in range(200):
            p.stepSimulation()


def create_movement_operations(path: ToolPath, robot: pi.RobotBase):
    """Returns a list of elementary operations to move the robot based
    on a given tool path and active endeffector.

    Args:
        path(ToolPath): input tool path

    Returns:
        elementary_operations(list): elementary operations to move robot
    """
    elementary_operations = []

    for position, orientation, _ in path:
        elementary_operations.append(
            lambda i=position, j=orientation:
            robot.set_endeffector_pose(i, j))

    return elementary_operations


if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file3 = os.path.join(dirname,
                              'Objects', 'FoFa', 'FoFa.urdf')

    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                             '--background_color_green=1 ' +
                                             '--background_color_blue=1')
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF(urdf_file3, [-2, 5, 0], useFixedBase=True, globalScaling=0.001)

    test_robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])

    test_robot.set_joint_position(({'q2': np.deg2rad(-15.0),
                                  'q3': np.deg2rad(-90.0)}))
    for _ in range(100):
        p.stepSimulation()

    # Setting the points
    pt_start, orn_start = test_robot.get_endeffector_pose()
    pt_1 = [1.5, -2, 2]
    orn_1 = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
    pt_2 = [1.5, 0, 2]
    orn_2 = p.getQuaternionFromEuler([-3*np.pi/2, 0, 0])
    pt_3 = [1.5, -1, 2]
    pt_4 = [2, -1, 2]
    pt_5 = [2.5, -1, 2]

    # Setting the colors

    color_1 = [1, 0, 0]
    color_2 = [0, 1, 0]
    color_3 = [0, 0, 1]
    color_4 = [1, 0, 1]
    color_5 = [1, 1, 0]
    color_6 = [0, 1, 1]
    color_7 = [1, 1, 1]
    color_8 = [0.5, 0.5, 0.5]

    # Seting the interpolation parameters
    samples = 100
    clockwise = True  # Circular Direction
    radius = 0.6
    axis = 2

    # Setting position
    linear_path = linear_interpolation(start_point=pt_start, end_point=pt_2,
                                       samples=samples,
                                       start_orientation=orn_start,
                                       end_orientation=orn_2)
    elementary_operations = create_movement_operations(linear_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 1: Linear path
    linear_path = linear_interpolation(start_point=pt_2, end_point=pt_1,
                                       samples=samples,
                                       start_orientation=orn_2,
                                       end_orientation=orn_1)
    elementary_operations = create_movement_operations(linear_path, test_robot)
    linear_path.draw(color=color_1)
    run_elementary_operations(elementary_operations)

    # # Path 2: Linear path
    # linear_path = linear_interpolation(start_point=pt_1, end_point=pt_2,
    #                                    samples=samples,
    #                                    start_orientation=orn_1,
    #                                    end_orientation=orn_2)

    # elementary_operations = create_movement_operations(linear_path, test_robot)
    # linear_path.draw(color=[1, 1, 1])
    # run_elementary_operations(elementary_operations)

    # Path 3: Circular path
    circular_path = circular_interpolation(start_point=pt_1, end_point=pt_3,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_2)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    clockwise = False
    circular_path = circular_interpolation(start_point=pt_3, end_point=pt_2,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_3)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    axis = 0
    circular_path = circular_interpolation(start_point=pt_2, end_point=pt_3,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_4)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    axis = 0
    clockwise = True
    circular_path = circular_interpolation(start_point=pt_3, end_point=pt_1,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_5)
    run_elementary_operations(elementary_operations)

    # Path 1: Linear path
    linear_path = linear_interpolation(start_point=pt_1, end_point=pt_3,
                                       samples=samples,
                                       start_orientation=orn_1,
                                       end_orientation=orn_1)
    elementary_operations = create_movement_operations(linear_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    axis = 1
    clockwise = True
    circular_path = circular_interpolation(start_point=pt_3, end_point=pt_4,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_6)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    axis = 1
    clockwise = False
    circular_path = circular_interpolation(start_point=pt_4, end_point=pt_5,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    circular_path.draw(color=color_7)
    run_elementary_operations(elementary_operations)

    # Path 1: Linear path
    linear_path = linear_interpolation(start_point=pt_5, end_point=pt_1,
                                       samples=samples,
                                       start_orientation=orn_1,
                                       end_orientation=orn_1)
    elementary_operations = create_movement_operations(linear_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 1: Spline path

    spline_path = spline_interpolation(
        [pt_1, pt_4, pt_2], samples=samples)
    spline_path.draw(color=color_8, pose=True)
    spline_path.orientations = np.transpose([orn_1]
                                            * len(spline_path.orientations[0]))
    elementary_operations = create_movement_operations(
        spline_path, test_robot)
    run_elementary_operations(elementary_operations)
