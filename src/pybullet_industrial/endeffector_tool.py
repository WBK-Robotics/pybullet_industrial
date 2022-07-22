import pybullet as p
import numpy as np
from typing import List
from pybullet_industrial import RobotBase


class EndeffectorTool:
    def __init__(self, urdf_model: str, start_position, start_orientation,
                 coupled_robots: List[RobotBase] = None, tcp_frame=None, connector_frames=None):
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
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._link_name_to_index = {}
        self._coupled_robots = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf, joint_number)[
                12].decode("utf-8")
            self._link_name_to_index[link_name] = joint_number

        if tcp_frame is None:
            last_link = max(self._link_name_to_index)
            self._tcp_id = self._link_name_to_index[last_link]
        else:
            self._tcp_id = self._convert_link_to_id(tcp_frame)

        if connector_frames is None:
            self._connector_id = -1
            base_pos, base_ori = p.getBasePositionAndOrientation(self.urdf)
        else:
            self._connector_id = self._convert_link_to_id(connector_frames)
            link_state = p.getLinkState(self.urdf, self._connector_id)
            base_pos = link_state[0]
            base_ori = link_state[1]

        self._coupled_robot = None
        self._coupling_link = None

        self._coupling_constraint = p.createConstraint(self.urdf,
                                                       -1, -1, -1,
                                                       p.JOINT_FIXED,
                                                       [0, 0, 0],
                                                       [0, 0, 0],
                                                       start_position,
                                                       None,
                                                       start_orientation)

        if not coupled_robots is None:
            self.couple(coupled_robots)

        tcp_pos, tcp_ori = self.get_tool_pose(tcp_frame)

        base_pos_inv, base_ori_inv = p.invertTransform(base_pos, base_ori)
        self._tcp_translation, self._tcp_rotation = p.multiplyTransforms(
            base_pos_inv, base_ori_inv, tcp_pos, tcp_ori)

    def couple(self, robot, endeffector_name=None):
        """Dynamically Couples the Tool with the Endeffector of a given robot.
        Note that this endeffector can also be a virtual link to connect a sensor.
        A Tool can only be coupled with one robot

        Args:
            robot (wbk_sim.robot): The robot whith which the tool should couple.
            endeffector_name (str, optional): The endeffector of the robot
                                              where the tool should couple to.
                                              Defaults to None.

        Raises:
            Error: [description]
        """
        # creates a fixed constrained between the choosen robot and the tool
        if self._coupled_robot is not None:
            raise ValueError("The Tool is already coupled with a robot")
        else:
            if endeffector_name is None:
                endeffector_index = robot._default_endeffector_id
            else:
                endeffector_index = robot._convert_endeffector(
                    endeffector_name)
            self._coupled_robot = robot
            self._coupling_link = endeffector_name
            p.removeConstraint(self._coupling_constraint)
            self._coupling_constraint = p.createConstraint(self._coupled_robot.urdf, endeffector_index,
                                                           self.urdf, self._connector_id,
                                                           p.JOINT_FIXED,
                                                           [0, 0, 0],
                                                           [0, 0, 0],
                                                           [0, 0, 0])

    def is_coupled(self):
        """Function which returns true if the Tool is currently coupled to a robot

        Returns:
            bool: 1 if the tool is coupled, 0 if not
        """
        if self._coupled_robot is None:
            return 0
        else:
            return 1

    def decouple(self):
        """Decouples the tool from the current robot.
           In this case a new constraint is created rooting the tool in its current pose.
        """
        self._coupled_robot = None
        self._coupling_link = None
        p.removeConstraint(self._coupling_constraint)
        position, orientation = p.getBasePositionAndOrientation(self.urdf)
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                       -1, -1, -1,
                                                       p.JOINT_FIXED,
                                                       [0, 0, 0],
                                                       [0, 0, 0],
                                                       position,
                                                       None,
                                                       orientation)
        pass

    def get_tool_pose(self, tcp_frame: str = None):
        """Returns the pose of the tool center point.
           Using the tcp_frame argument the state of other links can also be returned

        Args:
            tcp_frame (str, optional): the name of the link whose pose should be returned.
                                       Defaults to None in which case the default tcp is used

        Returns:
            np.array: The 3D position the link the world coordinate system
            np.array: A quaternion describing the orientation of the link in world coordinates
        """
        if tcp_frame is None:
            tcp_id = self._tcp_id
        else:
            tcp_id = self._convert_link_to_id(tcp_frame)

        link_state = p.getLinkState(self.urdf, tcp_id)

        position = np.array(link_state[0])
        orientation = np.array(link_state[1])
        return position, orientation

    def set_tool_pose(self, target_position, target_orientation=None):
        """Allows the control of the tool.
           If the tool is coupled the inverse kinematic control of a coupled robot is used.
           If not the tool is moved directly.

        Args:
            target_position (_type_): the desired position of the tool center point (tcp)
            target_orientation (_type_, optional): the desired position of
                                                   the tool center point (tcp).
                                                   If none is provided only
                                                   the position of the robot is controlled.
        """

        if self.is_coupled():

            tcp_translation_inv, tcp_rotation_inv = p.invertTransform(
                self._tcp_translation, self._tcp_rotation)
            adj_target_position, adj_target_orientation = p.multiplyTransforms(
                target_position, target_orientation, tcp_translation_inv, tcp_rotation_inv)

            self._coupled_robot.set_endeffector_pose(
                adj_target_position, adj_target_orientation, endeffector_name=self._coupling_link)
        else:
            if target_orientation is None:
                _, adj_target_orientation = p.getBasePositionAndOrientation(
                    self.urdf)
            p.removeConstraint(self._coupling_constraint)
            self._coupling_constraint = p.createConstraint(self.urdf,
                                                           self._tcp_id, -1, -1,
                                                           p.JOINT_FIXED,
                                                           [0, 0, 0],
                                                           [0, 0, 0],
                                                           target_position,
                                                           None,
                                                           target_orientation)

    def apply_tcp_force(self, force, world_coordinates=True):
        """Function which can apply a external Force at a the next simulation step.

            Carefull, this does not behave as expected for setRealTimeSimulation(1)!

        Args:
            force ([type]): A 3 dimensional force vector in Newton.
            world_coordinates (bool, optional): Specify wheter the force is defined
                                                in the world coordinates or the relative link frame.
                                                Defaults to True.
        """
        if world_coordinates:
            position, _ = self.get_tool_pose()
            p.applyExternalForce(self.urdf, self._tcp_id,
                                 force, position, p.WORLD_FRAME)
        else:
            p.applyExternalForce(self.urdf, self._tcp_id,
                                 force, [0, 0, 0], p.LINK_FRAME)

    def apply_tcp_torque(self, torque):
        """Function which can apply a external Torque at a the next simulation step.
           The local tcp_link frames are used as the main torque axis.

            Carefull, this does not behave as expected for setRealTimeSimulation(1)!

        Args:
            torque ([type]): A 3 dimensional torque vector in Newtonmeter.
        """
        p.applyExternalTorque(self.urdf, self._tcp_id,
                              torque, p.LINK_FRAME)

    def _convert_link_to_id(self, tcp):
        """Internal function that converts between link names and pybullet specific indexes

        Args:
            tcp (str): the name of the tool center point link

        Raises:
            TypeError: If the provided object is not a string

        Returns:
            int: the pybullet specific index of the link
        """
        if isinstance(tcp, str):
            if tcp in self._link_name_to_index.keys():
                return self._link_name_to_index[tcp]
            else:
                ValueError("Invalid Link name! valid names are: " +
                           str(self._link_name_to_index.keys()))
        else:
            raise TypeError(
                "The Link name must be a String describing a URDF link")
