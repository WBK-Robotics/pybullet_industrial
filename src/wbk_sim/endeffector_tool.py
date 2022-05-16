from msilib.schema import Error
from typing import Dict

import numpy as np
import pybullet as p

class EndeffectorTool:
    def __init__(self,urdf_model: str, start_position, start_orientation,coupled_robot=None):
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._coupled_robot = coupled_robot
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                        -1, -1, -1,
                                                        p.JOINT_FIXED,
                                                        [0, 0, 0],
                                                        [0, 0, 0],
                                                        start_position,
                                                        start_orientation)
    def couple(self,robot,endeffector_name=None):
        """Dynamically Couples the Tool with the Endeffector of a given robot.
        Note that this endeffector can also be a virtual link to connect a sensor.
        A Tool can only be coupled with one robot

        Args:
            robot (wbk_sim.robot): [description]
            endeffector_name (str, optional): [description]. Defaults to None.

        Raises:
            Error: [description]
        """
        #creates a fixed constrained between the choosen robot and the tool
        if self._coupled_robot is not None:
            raise Error("The Tool is already coupled with a robot")
        else:
            if endeffector_name == None:
                endeffector_index = robot._default_endeffector_id
            else:
                endeffector_index = robot._convert_endeffector(endeffector_name)
            self._coupled_robot = robot

            p.removeConstraint(self._coupling_constraint)
            self._coupling_constraint = p.createConstraint(self._coupled_robot.urdf, endeffector_index,
                                                           self.urdf, -1,
                                                           p.JOINT_FIXED,
                                                          [0, 0, 0],
                                                          [0, 0, 0],
                                                          [0, 0, 0],
                                                          [0, 0, 0])

    def is_coupled(self):
        if self._coupled_robot is None:
            return 0
        else:
            return 1

    def decouple(self):
        self._coupled_robot = None
        p.removeConstraint(self._coupling_constraint)
        position, orientation = p.getBasePositionAndOrientation(self.urdf)
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                        -1, -1, -1,
                                                        p.JOINT_FIXED,
                                                        [0, 0, 0],
                                                        [0, 0, 0],
                                                        position,
                                                        orientation)
        pass


    def get_tool_pose(self):
         #converts between the robots forward kinematics and the tool forward kinematics
        pass

    def set_tool_pose(self,target_position,target_orientation=None):
        #converts between the robots inverse kinematics and the tool inverse kinematics
        # Assumes the currently connected endeffector for the robot inv kin
        pass


if __name__ == "__main__":
    import os
    import time
    import pybullet as p
    import wbk_sim as wbk
    import numpy as np
    dirname = os.path.dirname(__file__)
    parentDir = os.path.dirname(dirname)
    parentDir = os.path.dirname(parentDir)
    urdf_file1 = os.path.join( parentDir,'examples','robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(parentDir,'examples','robot_descriptions', 'camera.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1,[0,0,0],start_orientation)

    tool = EndeffectorTool(urdf_file2,[1.9,0,1.2],start_orientation)

    

    p.setRealTimeSimulation(1)

    target_position = [1.9,0,1.2]
    time_step = 0.01
    while True:
        for i in range(300): 
            target_orientation = p.getQuaternionFromEuler([0, i/100, 0])  
            robot.set_endeffector_pose(target_position,target_orientation) 
            time.sleep(time_step) 

        if not tool.is_coupled():
            tool.couple(robot,'link6')
        else:
            tool.decouple()