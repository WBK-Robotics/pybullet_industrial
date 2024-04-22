import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np


def actuate_gripper(gripper: pi.Gripper, val: int):
    return gripper.actuate(val)


def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase, link: chr):
    return gripper.couple(robot, link)


def decouple_endeffector(gripper: pi.Gripper):
    return gripper.decouple()


if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'gripper_cad.urdf')
    urdf_file3 = os.path.join(dirname,
                              'Objects', 'FoFa', 'FoFa.urdf')

    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                             '--background_color_green=1 ' +
                                             '--background_color_blue=1')
    p.setPhysicsEngineParameter(numSolverIterations=10000)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    p.loadURDF(urdf_file3, [-2, 5, 0], useFixedBase=True, globalScaling=0.001)

    test_robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])

    test_robot.set_joint_position(({'q2': np.deg2rad(-15.0),
                                  'q3': np.deg2rad(-90.0)}))
    for _ in range(100):
        p.stepSimulation()

    start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
    test_gripper = pi.Gripper(urdf_file2, [2.7, -0.5, 1.2], start_orientation)
    endeffector_list = []
    endeffector_list.append(test_gripper)

    # M-Commands have to be added in this convention
    m_commands = {
        "10": [lambda: actuate_gripper(test_gripper, 1)],
        "11": [lambda: actuate_gripper(test_gripper, 0)]
    }

    # T-Commands have to be added in this convention
    t_commands = {
        "0": [lambda: decouple_endeffector(test_gripper)],
        "1": [lambda: couple_endeffector(test_gripper, test_robot, 'link6')]
    }

    dirname = os.path.dirname(__file__)
    textfile = os.path.join(dirname, 'g_codes', 'g_code_processor_example.txt')

    with open(textfile, encoding='utf-8') as f:
        gcode_input = f.read()

    demonstration_object = pi.GCodeProcessor(gcode_input, test_robot,
                                             endeffector_list,
                                             m_commands, t_commands)

    # Create an iterator from the demonstration object
    demonstration_iterator = iter(demonstration_object)

    # Iterate over the demonstration object
    for _ in demonstration_iterator:
        # Execute the simulation steps
        for _ in range(200):
            p.stepSimulation()
