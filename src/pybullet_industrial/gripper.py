from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool


class Gripper(EndeffectorTool):
    def __init__(self, urdf_model: str, start_position, start_orientation, coupled_robots = None, tcp_frame=None, connector_frames=None):
        """The base class for all Tools and Sensors connected to a Robot

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            coupled_robot ([type], optional): A wbk_sim.Robot object if
                                              the robot is coupled from the start.
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link
                                          describing the tool center point.
                                          Defaults to None in which case the last link is used.
            connector_frame ([type], optional): The name of the urdf_link
                                                at which a robot connects.
                                                Defaults to None in which case the base link is used.
        """
        super().__init__(urdf_model, start_position, start_orientation, coupled_robots, tcp_frame, connector_frames)


        self._gripper_constraint = p.createConstraint( self.urdf,
                                                       1,
                                                       self.urdf,
                                                       2,
                                                       jointType=p.JOINT_GEAR,
                                                       jointAxis=[0, 0, 1],
                                                       parentFramePosition=[0, 0, 0],
                                                       childFramePosition=[0, 0, 0])

        p.changeConstraint(self._gripper_constraint, gearRatio=-1, erp=0.1, maxForce=50)
