import time
import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np


def actuate_gripper(gripper: pi.Gripper, val: int):

    gripper.actuate(val)
    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)


def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase, link: chr):

    gripper.couple(robot, link)
    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)


def decouple_endeffector(gripper: pi.Gripper):

    gripper.decouple()
    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'gripper_cad.urdf')

    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                             '--background_color_green=1 ' +
                                             '--background_color_blue=1')
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    test_robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    test_robot.set_joint_position(({'q2': np.deg2rad(-15.0),
                                  'q3': np.deg2rad(-90.0)}))
    for _ in range(100):
        p.stepSimulation()

    start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
    test_gripper = pi.Gripper(urdf_file2, [2.7, -0.5, 1.2], start_orientation)
    endeffector_list = []
    endeffector_list.append(test_gripper)

    # Erstellung von M-Befehlen
    m_commands = [[] for _ in range(100)]
    m_commands[10].append(lambda: actuate_gripper(test_gripper, 1))
    m_commands[11].append(lambda: actuate_gripper(test_gripper, 0))

    # Erstellung von T-Befehlen
    t_commands = [[] for _ in range(100)]
    t_commands[0].append(lambda: decouple_endeffector(test_gripper))
    t_commands[1].append(lambda: couple_endeffector(test_gripper,
                                                    test_robot, 'link6'))
    dirname = os.path.dirname(__file__)

    gcode_obj_1 = pi.Gcode_class(test_robot, endeffector_list,
                                 m_commands, t_commands)

    textfile = os.path.join(dirname, 'Gcodes', 'gcode_G0.txt')
    gcode = gcode_obj_1.read_gcode(textfile)
    gcode_obj_1.run_gcode(gcode)

    textfile = os.path.join(dirname, 'Gcodes', 'gcode_G123.txt')
    gcode = gcode_obj_1.read_gcode(textfile)
    gcode_obj_1.run_gcode(gcode)
