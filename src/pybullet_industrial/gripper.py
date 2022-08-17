from typing import Dict

import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool


class Gripper(EndeffectorTool):
    def __init__(self, urdf_model: str, start_position, start_orientation, coupled_robots=None, tcp_frame=None,
                 connector_frames=None):
        """The base class for all Tools and Sensors connected to a Robot

        Args:
            urdf_model (str): A valid path to a urdf file describing the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            coupled_robots ([type], optional): A wbk_sim.Robot object if
                                              the robot is coupled from the start.
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link
                                          describing the tool center point.
                                          Defaults to None in which case the last link is used.
            connector_frames ([type], optional): The name of the urdf_link
                                                at which a robot connects.
                                                Defaults to None in which case the base link is used.
        """
        super().__init__(urdf_model, start_position, start_orientation, coupled_robots, tcp_frame, connector_frames)

        self._joint_name_to_index = {}
        self._lower_joint_limit = np.zeros(p.getNumJoints(self.urdf))
        self._upper_joint_limit = np.zeros(p.getNumJoints(self.urdf))
        self._actuated_joints = []

        for joint_number in range(p.getNumJoints(self.urdf)):
            if p.getJointInfo(self.urdf, joint_number)[2] != 4:
                joint_name = p.getJointInfo(self.urdf, joint_number)[1].decode("utf-8")
                self._joint_name_to_index[joint_name] = joint_number
                self._actuated_joints.append(joint_number)

                lower_limit = p.getJointInfo(self.urdf, joint_number)[8]
                upper_limit = p.getJointInfo(self.urdf, joint_number)[9]
                if upper_limit < lower_limit:
                    lower_limit = -np.inf
                    upper_limit = np.inf
                self._lower_joint_limit[joint_number] = lower_limit
                self._upper_joint_limit[joint_number] = upper_limit

        self.max_joint_force = 20 * np.ones(p.getNumJoints(self.urdf))

        for joint_number in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

        self._gripper_constraints = []
        for i in range(1, len(self._actuated_joints)):
            self._gripper_constraints.append(p.createConstraint(self.urdf,
                                                                self._actuated_joints[0],
                                                                self.urdf,
                                                                self._actuated_joints[i],
                                                                jointType=p.JOINT_GEAR,
                                                                jointAxis=[1, 0, 0],
                                                                parentFramePosition=[0, 0, 0],
                                                                childFramePosition=[0, 0, 0]))

            p.changeConstraint(self._gripper_constraints[-1], gearRatio=-1, erp=0.1, maxForce=5000)
        for joint_number in self._actuated_joints:
            p.setJointMotorControl2(self.urdf, joint_number, p.VELOCITY_CONTROL, targetVelocity=0,
                                    force=self.max_joint_force[joint_number])

    def actuate(self, target: float):
        """Actuates the gripper.

        Args:
            target([float]): Relative amount of maximum travel distance the gripper should move
         """
        for joint_number in self._actuated_joints:
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=self._lower_joint_limit[joint_number] + (
                                            self._upper_joint_limit[joint_number]
                                            - self._lower_joint_limit[joint_number]) * target)


class SuctionGripper(EndeffectorTool):
    def __init__(self, urdf_model: str, start_position, start_orientation, coupled_robots=None, tcp_frame=None,
                 connector_frames=None, suction_links=None):
        """The base class for all Tools and Sensors connected to a Robot

        Args:
            urdf_model (str): A valid path to a urdf file describing the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            coupled_robots ([type], optional): A wbk_sim.Robot object if
                                              the robot is coupled from the start.
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link
                                          describing the tool center point.
                                          Defaults to None in which case the last link is used.
            connector_frames ([type], optional): The name of the urdf_link
                                                at which a robot connects.
                                                Defaults to None in which case the base link is used.
            suction_links ([type], optional): The names of the urdf_links wich represent the active suction parts.
                                              Defaults to all Links.
                                            
        """
        super().__init__(urdf_model, start_position, start_orientation, coupled_robots, tcp_frame, connector_frames)
        self.suction_constraints = []

        self._link_name_to_index = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf, joint_number)[
                12].decode("utf-8")
            self._link_name_to_index[link_name] = joint_number

        if suction_links is None:
            self._suction_links_ids = None
        else:
            for name in suction_links:
                self._suction_links_ids.append(self._link_name_to_index[name])

    def activate(self, tolerance=0.0001):
        """Function to activate the suction gripper--> creates constraints between gripper and object

        :param tolerance: tolerance of contacts, i.e. at which distance the contact point is considered relevant.
        :returns ids of coupled objects
        """
        contact_points = list(p.getContactPoints(self.urdf))
        positionG, orientationG = self.get_tool_pose()
        invPos, invOrn = p.invertTransform(positionG, orientationG)

        coupled_bodys = []

        i = 0
        while i < len(contact_points):
            if contact_points[i][2] is self.urdf or contact_points[i][2] is self._coupled_robot.urdf or contact_points[i][8]>tolerance:
                del contact_points[i]
            else:
                print(contact_points[i][2], contact_points[i][8])
                i = i+1
        print(contact_points)

        if self._suction_links_ids is not None:
            i = 0
            while i < len(contact_points):
                if contact_points[i][3] not in self._suction_links_ids:
                    del contact_points[i]
                else:
                    i = i + 1
        print(contact_points)

        for cp in contact_points:
            if cp[2] not in coupled_bodys:
                coupled_bodys.append(cp[2])


        for cb in coupled_bodys:
            positionO, orientationO = p.getBasePositionAndOrientation(cb)
            restraintPos, restraintOrn = p.multiplyTransforms(invPos, invOrn, positionO, orientationO)
            cid_grip = p.createConstraint(self.urdf, self._tcp_id,
                                          cb, -1,
                                          p.JOINT_FIXED, [0, 0, 0],
                                          parentFramePosition=restraintPos, childFramePosition=[0, 0, 0],
                                          parentFrameOrientation=restraintOrn, childFrameOrientation=None)
            self.suction_constraints.append(cid_grip)
        return coupled_bodys

    def deactivate(self):
        for sc in self.suction_constraints:
            p.removeConstraint(sc)
