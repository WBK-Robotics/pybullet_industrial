import os
import pybullet as p
import numpy as np
import pybullet_data
import pybullet_industrial as pi
from pybullet_industrial import linear_interpolation
from pybullet_industrial import circular_interpolation
from pybullet_industrial import ToolPath


def simulate_circular_path(robot: pi.RobotBase, end_point: np.array,
                           end_orientation: np.array,
                           color=None, samples=200, axis=2,
                           clockwise=True, radius=0.6):
    """Creates a linear path and calls the simulate function

    Args:
        robot (RobotBase): operating robot
        end_point(np.array): next point
        end_orientation(np.array): next orientation
        color: color for drawing the path
        samples: number of samples used to interpolate
        axis: The axis around which the circle is interpolated
        clockwise: direction of the interpolation
        radius: radius of the interpolation
    """
    start_point, start_orientation = robot.get_endeffector_pose()
    path = circular_interpolation(start_point=start_point, end_point=end_point,
                                  samples=samples, axis=axis,
                                  clockwise=clockwise, radius=radius,
                                  start_orientation=start_orientation,
                                  end_orientation=end_orientation)
    if color is not None:
        path.draw(color=color)
    simulate_path(path, robot)


def simulate_linear_path(robot: pi.RobotBase, end_point: np.array,
                         end_orientation: np.array,
                         color=None, samples=200):
    """Creates a linear path and calls the simulate function

    Args:
        robot (RobotBase): operating robot
        end_point(np.array): next point
        end_orientation(np.array): next orientation
        color: color for drawing the path
        samples: number of samples used to interpolate
    """
    start_point, start_orientation = robot.get_endeffector_pose()
    path = linear_interpolation(start_point=start_point, end_point=end_point,
                                samples=samples,
                                start_orientation=start_orientation,
                                end_orientation=end_orientation)
    if color is not None:
        path.draw(color=color)
    simulate_path(path, robot)


def simulate_path(path: ToolPath, robot: pi.RobotBase):
    """Simulates the created path by running the elementary operations

    Args:
        path (ToolPath): input tool path
        robot (RobotBase): operating robot
    """
    elementary_operations = create_movement_operations(path, robot)

    for operation in elementary_operations:
        operation()
        for _ in range(100):
            p.stepSimulation()


def create_movement_operations(path: ToolPath, robot: pi.RobotBase):
    """Returns a list of elementary operations to move the robot based
    on a given tool path and active endeffector.

    Args:
        path (ToolPath): input tool path
        robot (RobotBase): operating Robot

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
    pt_1 = [1.8, -2, 2]
    pt_2 = [1.8, 0, 2]
    pt_3 = [1.8, -1, 2]
    pt_4 = [2.1, -1, 2]
    pt_5 = [2.4, -1, 2]

    # Setting the orientations
    orn_1 = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
    orn_2 = p.getQuaternionFromEuler([-np.pi/2, -np.pi/2, 0])
    orn_3 = p.getQuaternionFromEuler([-np.pi/2, -np.pi/2, -np.pi/2])
    orn_4 = p.getQuaternionFromEuler([-np.pi, 0, 0])

    # Setting the colors
    color_1 = [1, 0, 0]
    color_2 = [0, 1, 0]
    color_3 = [0, 0, 1]
    color_4 = [1, 0, 1]
    color_5 = [1, 1, 0]
    color_6 = [0, 1, 1]
    color_7 = [1, 1, 1]
    color_8 = [0.5, 0.5, 0.5]

    # Setting position
    simulate_linear_path(test_robot, pt_5, orn_1)

    # Move to point 2
    simulate_linear_path(
        robot=test_robot, end_point=pt_2, end_orientation=orn_2)

    # Path 1: Linear path
    simulate_linear_path(robot=test_robot, end_point=pt_1,
                         end_orientation=orn_1, color=color_1)

    # Path 2: Circular path
    simulate_circular_path(test_robot, pt_3, orn_1, color=color_2)

    # Path 3: Circular path
    simulate_circular_path(test_robot, pt_2, orn_4,
                           color=color_3, clockwise=False)

    # Path 4: Circular path
    simulate_circular_path(test_robot, pt_3, orn_1, color=color_4, axis=0)

    # Path 5: Circular path
    simulate_circular_path(test_robot, pt_1, orn_2,
                           color=color_5, clockwise=False, axis=0)

    # Path 6: Linear path
    simulate_linear_path(test_robot, pt_3, orn_1)

    # Path 7: Circular path
    simulate_circular_path(test_robot, pt_4, orn_2,
                           color=color_6, axis=1)

    # Path 8: Circular path
    simulate_circular_path(test_robot, pt_5, orn_1,
                           color=color_7, axis=1, clockwise=False)

    # Path 9: Linear path
    simulate_linear_path(test_robot, pt_1, orn_2)
