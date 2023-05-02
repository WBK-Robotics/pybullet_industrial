import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np
from g_code_processor import GCodeProcessor


def read_gcode(filename: str):
    """Reads G-Code row by row and saves the processed Data in
    a List.
    Comments that start with % are ignored and all the other data is
    stored as it gets read in.
    Every Line in the G-Code resembles the same structure as the text file

    Args:
        filename (str): Source of information

    Returns:
        gcode
    """
    with open(filename, encoding='utf-8') as f:
        gcode = []

        # Loop over the lines of the file
        for line in f.readlines():

            # Initialize a new line as a list
            new_line = []

            # Read in G-Code if line is not a comment and not empty
            if line[0] != "%" and len(line) > 1:

                # Split the line into its components
                data = line.split()

                # Loop over the components
                for i in data:
                    # Determine the ID of the component
                    id_val = i[0]

                    # Extract the value of the component
                    val2 = float(i[1:])

                    if id_val in ["G", "M", "T"]:
                        # Insert the value into the corresponding
                        # column of the new line
                        new_line.append([id_val, int(val2)])
                    else:
                        new_line.append([id_val, val2])

                # Add the finished line to the list
                gcode.append(new_line)

        return gcode


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
    textfile = os.path.join(dirname, 'Gcodes', 'gcode_G123.txt')
    gcode = read_gcode(textfile)
    gcode_obj_1 = GCodeProcessor(gcode, test_robot, endeffector_list,
                                 m_commands, t_commands)

    for gcode_line in gcode_obj_1:
        for _ in range(20):
            gcode_line
            for _ in range(10):
                p.stepSimulation()
