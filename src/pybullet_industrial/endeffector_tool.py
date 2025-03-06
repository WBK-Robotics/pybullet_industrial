import numpy as np
import pybullet as p

from pybullet_industrial import RobotBase


class EndeffectorTool:
    """The base class for all Tools and Sensors connected to a Robot

    Args:
        urdf_model (str): A valid path to a urdf file describint the tool geometry
        start_position (np.array): the position at which the tool should be spawned
        start_orientation (np.array): the orientation at which the tool should be spawned
        coupled_robot (pi.RobotBase, optional): A pybullet_omdistrial.RobotBase object if
                                                the robot is coupled from the start.
                                                Defaults to None.
        tcp_frame (str, optional): The name of the urdf_link
                                   describing the tool center point.
                                   Defaults to None in which case the last link is used.
        connector_frame (str, optional): The name of the urdf_link at which a robot connects.
                                         Defaults to None in which case
                                         the base link is used.
    """

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 coupled_robot: RobotBase = None, tcp_frame: str = None,
                 connector_frame: str = None):

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

        if connector_frame is None:
            self._connector_id = -1
            base_pos, base_ori = p.getBasePositionAndOrientation(self.urdf)
        else:
            self._connector_id = self._convert_link_to_id(connector_frame)
            link_state = p.getLinkState(self.urdf, self._connector_id)
            base_pos = link_state[0]
            base_ori = link_state[1]

        self._tcp_translation, self._tcp_rotation = p.multiplyTransforms(
            *p.invertTransform(base_pos, base_ori), *self.get_tool_pose(tcp_frame))

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

        if coupled_robot is not None:
            self.couple(coupled_robot)

    def couple(self, robot: RobotBase, endeffector_name: str = None):
        """Dynamically Couples the Tool with the Endeffector of a given robot.
        Note that this endeffector can also be a virtual link to connect a sensor.
        A Tool can only be coupled with one robot

        Args:
            robot (pybullet_industrial.RobotBase): The robot whith which the tool should couple.
            endeffector_name (str, optional): The endeffector of the robot
                                              where the tool should couple to.
                                              Defaults to None.

        Raises:
            ValueError: If the tool is already coupled.
            TypeError: if the object to couple is nof of class RobotBase.
        """
        if self._coupled_robot is not None:
            raise ValueError("The tool is already coupled with a robot")
        if not isinstance(robot, RobotBase):
            raise TypeError(
                "A EndeffectorTool can only couple with a RobotBase object")

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

    def set_base_offset(self, base_offset: np.array):
        self.base_offset = base_offset

    def set_moving_object_offset(self, offset: np.array):
        self.moving_object_offset = offset

    def match_moving_object(self, moving_object):
        # Retrieve the current tool pose
        position, orientation = self.get_tool_pose()

        # Assume self.moving_object_offset is a tuple (position_offset, orientation_offset)
        pos_offset, ori_offset = self.moving_object_offset

        # Compose the tool pose with the offset (using multiplyTransforms ensures correct rotation composition)
        target_position, target_orientation = p.multiplyTransforms(
            position, orientation, pos_offset, ori_offset
        )

        # Update the moving object with the new pose
        p.resetBasePositionAndOrientation(
            moving_object, target_position, target_orientation)

    def match_endeffector_pose(self, robot: RobotBase):
        """
        Matches the tool's connector frame with the robot's end effector pose,
        optionally applying an additional offset to the tool's base pose.

        This method computes the new base pose for the tool so that the connector frame
        (as defined by `connector_frame` during initialization) aligns with the robot's
        end effector pose (retrieved via robot.get_endeffector_pose()). An additional offset
        (position and orientation) can be applied to the resulting base pose.

        Args:
            robot (RobotBase): The robot whose end effector pose should be matched.
            base_offset (tuple): A tuple (position_offset, orientation_offset) where:
                - position_offset (list or np.array): 3D translation offset.
                - orientation_offset (list or np.array): Quaternion [x, y, z, w] offset.
                Defaults to ([0, 0, 0], [0, 0, 0, 1]) (i.e. no offset).
        """
        # Retrieve the robot's end effector pose.
        ee_pos, ee_ori = robot.get_endeffector_pose()

        # Get the current base pose of the tool.
        base_pos, base_ori = p.getBasePositionAndOrientation(self.urdf)

        # Compute the transform from the tool's base to its connector frame.
        if self._connector_id == -1:
            # If no connector frame is specified, assume the base IS the connector.
            base_to_connector_translation = [0, 0, 0]
            base_to_connector_rotation = [0, 0, 0, 1]
        else:
            # Get the connector's current pose.
            connector_state = p.getLinkState(self.urdf, self._connector_id)
            connector_pos, connector_ori = connector_state[0], connector_state[1]
            base_to_connector_translation, base_to_connector_rotation = p.multiplyTransforms(
                *p.invertTransform(base_pos, base_ori),
                connector_pos, connector_ori
            )

        # Invert the base-to-connector transform.
        inv_trans, inv_rot = p.invertTransform(
            base_to_connector_translation, base_to_connector_rotation)

        # Compute the new base pose
        new_base_pos, new_base_ori = p.multiplyTransforms(
            ee_pos, ee_ori,
            inv_trans, inv_rot
        )

        # Apply the additional base offset.
        pos_offset, ori_offset = self.base_offset
        final_base_pos, final_base_ori = p.multiplyTransforms(
            new_base_pos, new_base_ori,
            pos_offset, ori_offset
        )

        # Reset the tool's base pose so that its connector now matches the adjusted end effector pose.
        p.resetBasePositionAndOrientation(
            self.urdf, final_base_pos, final_base_ori)

    def set_tool_pose(self, target_position: np.array, target_orientation: np.array = None):
        """Allows the control of the tool.
           If the tool is coupled the inverse kinematic control of a coupled robot is used.
           If not the tool is moved directly.

        Args:
            target_position (np.array): the desired position of the tool center point (tcp)
            target_orientation (np.array, optional): the desired position of
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
            # Added couplin like behavior
            # p.resetBasePositionAndOrientation(
            #     self.urdf, adj_target_position, adj_target_orientation)
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

    def apply_tcp_force(self, force: np.array, world_coordinates: bool = True):
        """Function which can apply a external Force at a the next simulation step.

            Carefull, this does not behave as expected for setRealTimeSimulation(1)!

        Args:
            force (np.array): A 3 dimensional force vector in Newton.
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

    def apply_tcp_torque(self, torque: np.array):
        """Function which can apply a external Torque at a the next simulation step.
           The local tcp_link frames are used as the main torque axis.

            Carefull, this does not behave as expected for setRealTimeSimulation(1)!

        Args:
            torque (np.array): A 3 dimensional torque vector in Newtonmeter.
        """
        p.applyExternalTorque(self.urdf, self._tcp_id,
                              torque, p.LINK_FRAME)

    def _convert_link_to_id(self, tcp: str):
        """Internal function that converts between link names and pybullet specific indexes

        Args:
            tcp (str): the name of the tool center point link

        Raises:
            TypeError: If the provided object is not a string

        Returns:
            int: the pybullet specific index of the link
        """
        if not isinstance(tcp, str):
            raise TypeError(
                "The Link name must be a String describing a URDF link")
        if not tcp in self._link_name_to_index:
            raise ValueError("Invalid Link name! valid names are: " +
                             str(self._link_name_to_index.keys()))

        return self._link_name_to_index[tcp]
