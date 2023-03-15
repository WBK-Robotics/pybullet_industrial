import os
import unittest

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
from gcode_class import *

def test_final_position(robot: pi.RobotBase, target_position, target_orientation,  pos_precision, ori_precision):
    within_precision = True
    
    current_position, current_orientation = robot.get_endeffector_pose()
    current_orientation = np.array(p.getEulerFromQuaternion(current_orientation))
    position_error = np.linalg.norm(current_position-target_position)
    orientation_error = np.linalg.norm(
            current_orientation-target_orientation)
    print("Orientation_error: " , orientation_error)
    within_precision = within_precision and (
            position_error <= pos_precision) and (orientation_error <= ori_precision)

    return within_precision

def create_command(gcode_object : Gcode_class, commands):
    text_gcode = "text_gcode.txt"
    with open(text_gcode, "w") as f:
        f.writelines(commands)
    g_code = gcode_object.read_gcode(text_gcode)
    os.remove(text_gcode)
    return g_code

class Test_Gcode_class(unittest.TestCase):
    
    def test_read_gcode(self):
        dirname = os.path.dirname(__file__)
        textfile = os.path.join(dirname,'test_code.txt')
        test_objetct = Gcode_class()
        gcode = test_objetct.read_gcode(textfile)
        
        # Check that the length of the gcode list is correct
        self.assertEqual(len(gcode), 3)

        # Check that the first line is correct
        self.assertEqual(gcode[0], [['G', 0], ['X', 1.0], ['Y', 2.0]])

        # Check that the second line is correct
        self.assertEqual(gcode[1], [['G', 1], ['X', 2.0], ['Y', 3.0]])

        # Check that the third line is correct
        self.assertEqual(gcode[2], [['G', 2], ['X', 3.0], ['Y', 4.0], ['Z', 5.0]])
        
        
    def test_run_gcode(self):
        
        dirname = os.path.dirname(__file__)
        urdf_file1 = os.path.join(dirname,
                                'robot_descriptions', 'comau_nj290_robot.urdf')
       
        p.connect(p.GUI, options='--background_color_red=1 ' +
                '--background_color_green=1 ' +
                '--background_color_blue=1')
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
        test_object = Gcode_class()
        
      
        
        
        pos1 = [1.9,-0.5,1.5]
        ori1 =  [-1.57079, 0, 0]
        
        pos2 = [2.2,-0.3,1.6]
        ori2 =  [-1.57079, 0, 0]
       
        pos_precision = 0.02
        ori_precision = 0.004

        ########
        #Test G0
        
        #Create Textfile and run the commands
        cmd1 = "G0 X1.9 Y-0.5 Z1.5 A-1.5707 B0 C0"
        commands = [cmd1]
        g_code = create_command(test_object, commands)   
        test_object.run_gcode(g_code, robot)

        self.assertTrue(test_final_position(robot, pos1, ori1, pos_precision, ori_precision))

        cmd1 = "G1 X2.2 Y-0.3 Z1.6 A-1.5707 B0 C0"
        commands = [cmd1]
        g_code = create_command(test_object, commands)   
        test_object.run_gcode(g_code, robot)

        self.assertTrue(test_final_position(robot, pos2, ori2, pos_precision, ori_precision))

        
if __name__ == '__main__':
    unittest.main()