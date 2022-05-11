import os
import unittest
import numpy as np
import pybullet as p

import wbk_sim as wbk


class TestRobotBase(unittest.TestCase):
    def test_joint_interface(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        urdf_file1 = os.path.join( parentDir,'src','wbk_sim','robot_descriptions', 'comau_NJ290_3-0_m.urdf')
        print(urdf_file1)

        physics_client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        robot = wbk.RobotBase(urdf_file1,[0,0,0],start_orientation)

        p.createConstraint(robot.urdf,
                        -1, -1, -1,
                        p.JOINT_FIXED,
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0])

        precision = 10**-4 # 0.1 milimeter precision
        within_precision = True 
        for i in range(100):
            oscillation = np.sin(i/20)
            target_state = {'q1':oscillation,'q2':oscillation,'q3':oscillation*0.4-0.4,'q4':oscillation,'q5':oscillation,'q6':oscillation}
            robot.set_joint_position(target_state)
            for i in range(100):
                p.stepSimulation()
            
            actual_state = robot.get_joint_state()
            q1_error = target_state['q1']-actual_state['q1']['position']
            q2_error = target_state['q2']-actual_state['q2']['position']
            q3_error = target_state['q3']-actual_state['q3']['position']
            q4_error = target_state['q4']-actual_state['q4']['position']
            q5_error = target_state['q5']-actual_state['q5']['position']
            q6_error = target_state['q6']-actual_state['q6']['position']

            within_precision = within_precision and ((q1_error <= precision) and 
                                                     (q2_error <= precision) and 
                                                     (q3_error <= precision) and 
                                                     (q4_error <= precision) and 
                                                     (q5_error <= precision) and
                                                     (q6_error <= precision))

        self.assertTrue(within_precision)

if __name__ == '__main__':
    unittest.main()
