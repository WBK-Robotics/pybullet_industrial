import os
import unittest
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi


def couple_endeffector(gripper: pi.Gripper, robot: pi.RobotBase, link: chr):
    gripper.couple(robot, link)


def decouple_endeffector(gripper: pi.Gripper):
    gripper.decouple()


def run_simulation(iterator: pi.GCodeProcessor):

    for _ in iterator:
        for _ in range(200):
            p.stepSimulation()


def check_robot_position(robot: pi.RobotBase, target_position,
                         target_orientation,  pos_tol, ori_tol):

    within_precision = True

    current_position, current_orientation = robot.get_endeffector_pose()
    current_orientation = np.array(
        p.getEulerFromQuaternion(current_orientation))

    position_error = np.linalg.norm(current_position-target_position)
    orientation_error = np.linalg.norm(current_orientation -
                                       target_orientation)

    within_precision = within_precision and (
        position_error <= pos_tol) and (orientation_error <= ori_tol)

    return within_precision


def check_tool_position(endeffetor: pi.EndeffectorTool, target_position,
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


class Test_GCodeProcessor(unittest.TestCase):

    def test_read_g_code(self):

        # Create test textfile
        cmd1 = "% This is a comment\n"
        cmd2 = "G0 X1.0 Y2.0\n"
        cmd3 = "G1 X2.0 Y3.0\n"
        cmd4 = "G2 X3.0 Y4.0 Z5.0"
        g_code_input = cmd1 + cmd2 + cmd3 + cmd4

        # Creating the Test Object
        test_object = pi.GCodeProcessor(g_code_input)

        # Check that the length of the gcode list is correct
        self.assertEqual(len(test_object.g_code), 3)

        # Check that the first line is correct

        self.assertEqual(test_object.g_code[0], {'G': 0, 'X': 1.0, 'Y': 2.0})

        # Check that the second line is correct
        self.assertEqual(test_object.g_code[1], {'G': 1, 'X': 2.0, 'Y': 3.0})

        # Check that the third line is correct
        self.assertEqual(test_object.g_code[2], {
                         'G': 2, 'X': 3.0, 'Y': 4.0, 'Z': 5.0})

    def test_tool_path_to_g_code(self):

        positions = np.array([[1.0, 2.0],
                              [1.5, 2.5],
                              [2.0, 3.0]])
        orientations = np.array([[0, 0],
                                [0, 0],
                                [0, 0],
                                [1, 1]])
        tool_activations = np.array([False, True])
        tool_path = pi.ToolPath(positions, orientations, tool_activations)
        g_code = pi.GCodeProcessor.tool_path_to_g_code(tool_path)
        expected = [
            {'G': 1, 'X': 1.0, 'Y': 1.5, 'Z': 2.0,
             'A': 0.0, 'B': 0.0, 'C': 0.0},
            {'G': 1, 'X': 2.0, 'Y': 2.5, 'Z': 3.0,
             'A': 0.0, 'B': 0.0, 'C': 0.0}
        ]
        self.assertEqual(len(g_code), 2)
        self.assertEqual(g_code, expected)

    def test_joint_path_to_g_code(self):

        joint_order = ["q1", "q2", "q3"]
        joint_values = np.array([[0.1, 0.2],
                                [0.3, 0.4],
                                [0.5, 0.6]])
        joint_path = pi.JointPath(joint_values, joint_order)
        g_code = pi.GCodeProcessor.joint_path_to_g_code(joint_path)
        expected = [
            {'G': 1, 'RA1': 0.1, 'RA2': 0.3, 'RA3': 0.5},
            {'G': 1, 'RA1': 0.2, 'RA2': 0.4, 'RA3': 0.6},
        ]
        self.assertEqual(len(g_code), 2)
        self.assertEqual(g_code, expected)

    def test_simulation(self):

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
        test_object = pi.GCodeProcessor(robot=robot)
        test_iterator = iter(test_object)

        pos1 = np.array([1.9, -0.5, 1.5])
        ori1 = np.array([-1.5079, 0.0, 0.0])

        pos2 = np.array([2.2, -0.3, 1.6])
        ori2 = np.array([-1.403, 0.0, 0.0])

        pos_precision = 0.02
        ori_precision = 0.04

        # Test 1: G0 without coupled tool
        command = "G0 X1.9 Y-0.5 Z1.5 A-1.5079 B0 C0"
        test_object.g_code = test_object.read_g_code(command)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        # Test 2.1: G1 without coupled tool
        cmd1 = "G1 X2.2\n"
        cmd2 = "G1 Y-0.3 Z1.6 A-1.403"
        commands = cmd1+cmd2
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Test 2.2: G1 with Joint Position
        command = "G1 RA1=-0.26599832725015843 RA2=0.03251430369251254\
              RA3=-1.9721862154817666 RA4=-3.07545425948084\
                  RA5=-1.1542831213947777 RA6=-0.2932878229283541"
        test_object.g_code = test_object.read_g_code(command)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        command = "G1 RA1=-0.15645379114988892 RA2=0.2524070196026062 \
            RA3=-1.665587086827114 RA4=-2.967366576977744\
        RA5=-1.254545645841404 RA6=-0.2133265842224983"
        test_object.g_code = test_object.read_g_code(command)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Test 3: G54
        cmd1 = "G54\n"
        cmd2 = "G1 X-0.3 Y-0.2 Z-0.1 A-0.1049"
        commands = cmd1 + cmd2
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        ori2 = np.array([-1.5079, 0.0, 0.0])

        # Test 4: G500
        cmd1 = "G500\n"
        cmd2 = "G1 X2.2 Y-0.3 Z1.6"
        commands = cmd1 + cmd2
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Test 5: G2 with G19
        cmd1 = "G19\n"
        cmd2 = "G2 X1.9 Y-0.5 Z1.5 R1"
        commands = cmd1 + cmd2
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos1, ori1, pos_precision, ori_precision))

        # Test 6: G3 with G18
        cmd1 = "G18\n"
        cmd2 = "G3 X2.2 Y-0.3 Z1.6 R1\n"
        cmd3 = "G17"
        commands = cmd1 + cmd2 + cmd3
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(check_robot_position(
            robot, pos2, ori2, pos_precision, ori_precision))

        # Create Tool for testing
        urdf_file2 = os.path.join(parentDir, 'examples',
                                  'robot_descriptions', 'gripper_cad.urdf')
        start_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
        test_gripper = pi.Gripper(
            urdf_file2, [2.7, -0.5, 1.2], start_orientation)
        ori1 = np.array([-3.1, 0.0, 0.0])
        ori2 = np.array([-3.1, 0.0, 0.0])
        endeffector_list = []
        endeffector_list.append(test_gripper)

        # Create M-commands for testing
        test_var = []
        m_commands = {
            "1": [lambda: test_var.append(1)]
        }

        # Create T-commands for testing
        t_commands = {
            "0": [lambda: decouple_endeffector(test_gripper)],
            "1": [lambda: couple_endeffector(test_gripper, robot, 'link6')]
        }

        # Adapt Test Object
        test_object.endeffector_list = endeffector_list
        test_object.m_commands = m_commands
        test_object.t_commands = t_commands

        # Test 7: T1-/M- command and G0-command with tool coupled
        cmd1 = "T1\n"
        cmd2 = "G0 X1.9 Y-0.5 Z1.5 A-3.1 B0 C0\n"
        cmd3 = "M1"
        commands = cmd1 + cmd2 + cmd3
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(test_gripper.is_coupled())
        self.assertTrue(check_tool_position(test_gripper, pos1,
                        ori1, pos_precision, ori_precision))
        self.assertTrue(len(test_var) == 1)

        # Test 8: G1-command with tool coupled and T0-command for decoupling
        cmd1 = "G1 X2.2 Y-0.3 Z1.6\n"
        cmd2 = "T0 "
        commands = cmd1 + cmd2
        test_object.g_code = test_object.read_g_code(commands)
        run_simulation(test_iterator)

        self.assertTrue(not test_gripper.is_coupled())
        self.assertTrue(check_tool_position(test_gripper, pos2,
                        ori2, pos_precision, ori_precision))


if __name__ == '__main__':
    unittest.main()
