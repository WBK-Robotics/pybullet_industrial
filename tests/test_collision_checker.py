import os
import unittest
import pybullet_industrial as pi
import pybullet as p
import pybullet_data
import numpy as np
import tempfile


class Test_CollisionChecker(unittest.TestCase):


    def test_uppdate_g_code(self):
        dirname = os.path.dirname(__file__)
        parentDir = os.path.dirname(dirname)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        urdf_robot = os.path.join(
            parentDir, 'examples', 'robot_descriptions',
            'comau_nj290_robot.urdf')

        p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=10000)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        start_position = np.array([2.0, -6.5, 0])

        robot = pi.RobotBase(
            urdf_robot, start_position, start_orientation)
        robot.set_joint_position(
            {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})


if __name__ == '__main__':
    unittest.main()
