import os
import unittest

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
from gcode_class import *

def set_nan(test_array):
    test_array[:] = np.NaN
    test_array[0] = ""

def set_g(test_array, val):
    test_array[1] = val

def set_m(test_array, val):
    test_array[2] = val

def set_t(test_array, val):
    test_array[3] = val

def set_xyz(test_array, x,y,z):
    test_array[4] = x
    test_array[5] = y
    test_array[6] = z


def set_abc(test_array, a,b,c):
    test_array[7] = a
    test_array[8] = b
    test_array[9] = c

def set_ij(test_array, i, j):
    test_array[10] = i
    test_array[11] = j
    

def set_r(test_array, val):
    test_array[12] = val

def set_f(test_array, val):
    test_array[13] = val

class Test_Gcode_class(unittest.TestCase):
    
    def test_read_gcode(self):
        dirname = os.path.dirname(__file__)
        textfile = os.path.join(dirname,'test_code.txt')
        test_objetct = Gcode_class()
        input_array = test_objetct.read_gcode(textfile)
        
        #Check if Comments work 
        c = '%Kommentar\n'
        self.assertEqual(c,input_array[0,0])
        for i in input_array[0,1:]:
            self.assertTrue(np.isnan(i)) #Rest of the line has to be NaN
        #Test if Numbers are stored correctly
        test_array = np.array(['', 0, 1, 2, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0], dtype=object)
        self.assertTrue(all(test_array==input_array[1]))
        
        
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
        
        test_array = np.zeros(14, dtype='O')
        set_nan(test_array)
        set_g(test_array,1)
        set_xyz(test_array,1.9,-0.5,1.5)
        
        
       

        #Test G0
        ["",0,1,0,1.1,2.4,4,nan]
        
        #Tets G1
        #Test G2
        #Test G3
        #Test G54
        #Test G500
        #Test G17
        #Test G18
        #Test G19
        #Test M10
        #Test M11
        #Test Toolchange
        





if __name__ == '__main__':
    unittest.main()