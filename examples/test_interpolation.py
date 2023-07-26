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
    pt_1 = [2, -1.5, 1.5]
    orn_1 = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
    pt_2 = [2.3, 0, 2.4]
    orn_2 = p.getQuaternionFromEuler([-3*np.pi/2, 0, 0])

    # Seting the interpolation parameters
    samples = 100
    clockwise = True  # Circular Direction
    radius = 2
    axis = 2

    # Path 1: Linear path
    linear_path = linear_interpolation(start_point=pt_start, end_point=pt_1,
                                       samples=samples,
                                       start_orientation=orn_start,
                                       end_orientation=orn_1)
    elementary_operations = create_movement_operations(linear_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 2: Linear path
    linear_path = linear_interpolation(start_point=pt_1, end_point=pt_2,
                                       samples=samples,
                                       start_orientation=orn_1,
                                       end_orientation=orn_2)

    elementary_operations = create_movement_operations(linear_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 3: Circular path
    circular_path = circular_interpolation(start_point=pt_2, end_point=pt_1,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_2,
                                           end_orientation=orn_1)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 4: Circular path
    clockwise = False
    axis = 1
    circular_path = circular_interpolation(start_point=pt_1, end_point=pt_2,
                                           samples=samples, axis=axis,
                                           clockwise=clockwise, radius=radius,
                                           start_orientation=orn_1,
                                           end_orientation=orn_2)
    elementary_operations = create_movement_operations(
        circular_path, test_robot)
    run_elementary_operations(elementary_operations)

    # Path 5: Spline path
    spline_points = []
    spline_pt_1 = [1, -1.5, 2.4]
    spline_pt_2 = [2, -1.5, 2.4]
    spline_pt_3 = [2, -1.5, 1.5]
    spline_points = [spline_pt_1, spline_pt_2, spline_pt_3]

    spline_path = spline_interpolation(spline_points, samples=samples,
                                       start_orientation=orn_1,
                                       end_orientation=orn_2)
    elementary_operations = create_movement_operations(
        spline_path, test_robot)
    run_elementary_operations(elementary_operations)
