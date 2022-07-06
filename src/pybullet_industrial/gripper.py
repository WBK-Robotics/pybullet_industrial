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

        self._joint_state_shape = self.get_joint_state()
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

        self.max_joint_force = 20*np.ones(p.getNumJoints(self.urdf))

        for joint_number in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

        self._gripper_constraints =[]
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
            p.setJointMotorControl2(self.urdf, joint_number, p.VELOCITY_CONTROL, targetVelocity=0, force=self.max_joint_force[joint_number])


    def get_joint_state(self):
        """Returns the position of each joint as a dictionary keyed with their name

        Returns:
             Dict[str,Dict[str,float]]: The state of all joinst

        """
        joint_state = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            if p.getJointInfo(self.urdf, joint_number)[2] != 4:
                joint = p.getJointInfo(self.urdf, joint_number)[1].decode(
                    "utf-8")  # convert byte string to string
                joint_position = p.getJointState(self.urdf, joint_number)[0]
                joint_velocity = p.getJointState(self.urdf, joint_number)[1]
                joint_torque = p.getJointState(self.urdf, joint_number)[3]
                joint_reaction_force = p.getJointState(
                    self.urdf, joint_number)[2]

                single_joint_state = {'position': joint_position,
                                      'velocity': joint_velocity,
                                      'torque': joint_torque,
                                      'reaction force': joint_reaction_force}
                joint_state[joint] = single_joint_state
        return joint_state

    def set_joint_position(self,target: Dict[str,  float],ignore_limits=False):
        """Sets the target position for a number of joints.
           The maximum force of each joint is set according to the max_joint_force class attribute.

        Args:
            target (Dict[str,  float]): A dictionary containing the joint states to be set

        Raises:
            KeyError: If the specified joint state is not part of the Robot
        """
        if all(key in self._joint_state_shape.keys() for key in target.keys()):
            for joint, joint_position in target.items():
                joint_number = self._joint_name_to_index[joint]


                if ignore_limits == False:
                    lower_joint_limit = self._lower_joint_limit[joint_number]
                    upper_joint_limit = self._upper_joint_limit[joint_number]
                    if joint_position > upper_joint_limit or joint_position < lower_joint_limit:
                        raise ValueError('The joint position '+str(joint_position)+
                                        ' is aut of limit for joint '+joint+'. Its limits are:\n'+
                                        str(lower_joint_limit)+' and '+str(upper_joint_limit))


                p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                        force=self.max_joint_force[joint_number],
                                        targetPosition=joint_position)
        else:
            raise KeyError('One or more joints are not part of the gripper. ' +
                           'correct keys are: '+str(self._joint_state_shape.keys()))

    def actuate(self, target: float):
        """Actuates the gripper.

        Args:
            target([float]): Relative amount of maximum travel distance the gripper should move
         """
        for joint_number in self._actuated_joints:
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=self._lower_joint_limit[joint_number]+(self._upper_joint_limit[joint_number]
                                                    -self._lower_joint_limit[joint_number])*target)
