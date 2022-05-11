import os
import time
import pybullet as p
import wbk_sim as wbk

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    parentDir = os.path.dirname(dirname)
    urdf_file1 = os.path.join( parentDir,'src','wbk_sim','robot_descriptions', 'comau_NJ290_3-0_m.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1,[0,0,0],start_orientation)

    
    p.createConstraint(robot.urdf,
                       -1, -1, -1,
                       p.JOINT_FIXED,
                       [0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 0])
    

    p.setRealTimeSimulation(1)
    for i in range(1000): 
        target_orientation = p.getQuaternionFromEuler([0, i/100, 0]) 
        target_pose = [1.9,0,1.2] 
        robot.set_endeffector_pose('link6',target_pose,target_orientation,iterations=1) 
        time.sleep(0.1) 