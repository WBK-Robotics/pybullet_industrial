import os
import time
import pybullet as p
import wbk_sim as wbk
from lemniscate import build_lemniscate_path
import numpy as np


class TeachingController:
    def __init__(self):
        self.state = 0
        self.joint_traject


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,'robot_descriptions', 'comau_NJ290_3-0_m.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1,[0,0,0],start_orientation)

    

    p.setRealTimeSimulation(1)
    
    teaching_button = p.addUserDebugParameter("teaching",0,1,1)
    playback_speed = p.addUserDebugParameter("Playback Speed",1,20)


    teaching = True
    joint_trajectory = robot._joint_state_shape
    for key in joint_trajectory.keys():
        joint_trajectory[key]=[]
    
    while True:

        if teaching:
            if p.readUserDebugParameter(teaching_button) == 0:
                teaching = False
            else:
                joint_state = robot.get_joint_state()
                for key in joint_state.keys():
                    joint_trajectory[key].append(joint_state[key]['position'])
                time.sleep(0.01)

        if not teaching:
            if p.readUserDebugParameter(teaching_button) == 1:
                teaching = True
                robot.reset_robot([0,0,0],start_orientation)
                for key in joint_trajectory.keys():
                    joint_trajectory[key]=[]
            else:
                first_joint_trajectory = list(joint_trajectory.values())[0]
                for i in range(len(first_joint_trajectory)):
                    target_state = robot.get_joint_state()
                    for key in target_state.keys():
                        target_state[key]= joint_trajectory[key][i]
                    robot.set_joint_position(target_state,ignore_limits=True)
                    time.sleep(0.01/p.readUserDebugParameter(playback_speed))
