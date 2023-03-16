import os
import unittest
import time
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase, link: chr):
    gripper.couple(robot, 'link6')
    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)


def decouple_endeffector(gripper: pi.Gripper):
    gripper.decouple()
    for _ in range(25):
        p.stepSimulation()
        time.sleep(0.01)


def test_robot_position(robot: pi.RobotBase, target_position,
                        target_orientation,  pos_tol, ori_tol):
    within_precision = True
    current_position, current_orientation = robot.get_endeffector_pose()
    current_orientation = np.array(
        p.getEulerFromQuaternion(current_orientation))
    position_error = np.linalg.norm(current_position-target_position)
    orientation_error = np.linalg.norm(
        current_orientation-target_orientation)
    within_precision = within_precision and (
        position_error <= pos_tol) and (orientation_error <= ori_tol)
    return within_precision


def test_tool_position(endeffetor: pi.EndeffectorTool, target_position,
                       target_orientation,  pos_tol, ori_tol):
    within_precision = True
    current_position, current_orientation = endeffetor.get_tool_pose()
    current_orientation = np.array(
        p.getEulerFromQuaternion(current_orientation))
    position_error = np.linalg.norm(current_position-target_position)
    orientation_error = np.linalg.norm(
        current_orientation-target_orientation)
    within_precision = within_precision and (
        position_error <= pos_tol) and (orientation_error <= ori_tol)

    return within_precision


def create_command(gcode_object: pi.Gcode_class, commands):
    text_gcode = "text_gcode.txt"
    with open(text_gcode, "w") as f:
        f.writelines(commands)
    gcode = gcode_object.read_gcode(text_gcode)
    os.remove(text_gcode)
    return gcode


class Test_Gcode_class(unittest.TestCase):

    def test_read_gcode(self):
        # Create test textfile
        cmd1 = "% This is a comment\n"
        cmd2 = "G0 X1.0 Y2.0\n"
        cmd3 = "G1 X2.0 Y3.0\n"
        cmd4 = "G2 X3.0 Y4.0 Z5.0\n"
        commands = [cmd1, cmd2, cmd3, cmd4]
        gcode_object = pi.Gcode_class()
        gcode = create_command(gcode_object, commands)

        # Check that the length of the gcode list is correct
        self.assertEqual(len(gcode), 3)

        # Check that the first line is correct
        self.assertEqual(gcode[0], [['G', 0], ['X', 1.0], ['Y', 2.0]])

        # Check that the second line is correct
        self.assertEqual(gcode[1], [['G', 1], ['X', 2.0], ['Y', 3.0]])

        # Check that the third line is correct
        self.assertEqual(
            gcode[2], [['G', 2], ['X', 3.0], ['Y', 4.0], ['Z', 5.0]])

    def test_run_gcode(self):

        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file1 = os.path.join(parentDir, 'examples',
                                  'robot_descriptions',
                                  'comau_nj290_robot.urdf')

        # p.connect(p.GUI, options='--background_color_red=1 ' +
        #           '--background_color_green=1 ' +
        #           '--background_color_blue=1')
        p.connect(p.DIRECT)

        p.setPhysicsEngineParameter(numSolverIterations=10000)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setPhysicsEngineParameter(numSolverIterations=10000)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0, 0, -10)
        p.createCollisionShape(p.GEOM_MESH,
                               fileName="samurai_monastry.obj",
                               flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
        robot.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
        for _ in range(100):
            p.stepSimulation()

        dirname = os.path.dirname(__file__)
        test_object = pi.Gcode_class()

        pos1 = [1.9, -0.5, 1.5]
        ori1 = [-1.5709, 0, 0]

        pos2 = [2.2, -0.3, 1.6]
        ori2 = [-1.5709, 0, 0]

        pos_precision = 0.1
        ori_precision = 0.1

        # Test G0 without coupled tool
        cmd1 = "G0 X1.9 Y-0.5 Z1.5 A-1.5709 B0 C0"
        commands = [cmd1]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        # Test G1 without coupled tool
        cmd1 = "G1 X2.2 Y-0.3 Z1.6"
        commands = [cmd1]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Test G54
        cmd1 = "G54\n"
        cmd2 = "G1 X-0.2 Y-0.2 Z-0.1"
        commands = [cmd1, cmd2]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        # Test G500
        cmd1 = "G00\n"
        cmd2 = "G1 X2.2 Y-0.3 Z1.6"
        commands = [cmd1, cmd2]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Test G2
        cmd1 = "G19\n"
        cmd2 = "G2 X1.9 Y-0.5 Z1.5 R0.4"
        commands = [cmd1, cmd2]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        # Test G3
        cmd1 = "G18\n"
        cmd2 = "G3 X2.2 Y-0.3 Z1.6 R0.4\n"
        cmd3 = "G17"
        commands = [cmd1, cmd2, cmd3]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(g_code, robot)
        self.assertTrue(test_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Create Tool for testing
        urdf_file2 = os.path.join(parentDir, 'examples',
                                  'robot_descriptions', 'gripper_cad.urdf')
        start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
        test_gripper = pi.Gripper(
            urdf_file2, [2.7, -0.5, 1.2], start_orientation)
        ori1 = [np.pi, 0, 0]
        ori2 = [np.pi, 0, 0]
        endeffector_list = []
        endeffector_list.append(test_gripper)

        # Create M-commands for testing
        m_befehle = [[] for _ in range(100)]
        test_var = []
        m_befehle[1].append(lambda: test_var.append(1))

        # Create T-commands for testing
        t_befehle = [[] for _ in range(100)]
        t_befehle[0].append(lambda: decouple_endeffector(test_gripper))
        t_befehle[1].append(lambda: couple_endeffector(
            test_gripper, robot, 'link6'))

        # Test T1/M command and G0 command with tool coupled
        cmd1 = "T1\n"
        cmd2 = "G0 X1.9 Y-0.5 Z1.5\n"
        cmd3 = "M1"
        commands = [cmd1, cmd2, cmd3]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(
            g_code, robot, endeffector_list, m_befehle, t_befehle)
        self.assertTrue(test_gripper.is_coupled())
        self.assertTrue(test_tool_position(test_gripper, pos1,
                        ori1, pos_precision, ori_precision))
        self.assertTrue(len(test_var) == 1)

        # Test G1 command with tool coupled and T0 command for decoupling
        cmd1 = "G1 X2.2 Y-0.3 Z1.6\n"
        cmd2 = "T0 "
        commands = [cmd1, cmd2]
        g_code = create_command(test_object, commands)
        test_object.run_gcode(
            g_code, robot, endeffector_list, m_befehle, t_befehle)
        self.assertTrue(not test_gripper.is_coupled())
        self.assertTrue(test_tool_position(test_gripper, pos2,
                        ori2, pos_precision, ori_precision))


if __name__ == '__main__':
    unittest.main()
