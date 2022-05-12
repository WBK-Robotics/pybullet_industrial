from msilib.schema import Error
from typing import Dict

import numpy as np
import pybullet as p

class EndeffectorTool:
    def __init__(self,urdf_model: str, start_position, start_orientation,coupled_robot):
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._coupled_robot = coupled_robot
    def couple(self,robot,endeffector_name=None):
        #creates a fixed constrained between the choosen robot and the tool
        if self._coupled_robot is not None:
            raise Error("The Tool is already coupled with a robot")
        pass

    def decouple(self,robot):
         #deletes the constrained between the choosen robot and the tool
        pass


    def get_tool_pose(self):
         #converts between the robots forward kinematics and the tool forward kinematics
        pass

    def set_tool_pose(self,target_position,target_orientation=None):
        #converts between the robots inverse kinematics and the tool inverse kinematics
        # Assumes the currently connected endeffector for the robot inv kin
        pass
