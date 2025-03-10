from typing import Dict

import numpy as np
import pybullet as p


class RobotBase:
    """A Base class encapsulating a URDF based industrial robot manipulator

    Args:
        urdf_model (str): A valid path to a urdf file
        start_position (np.array): The start position of the robot base
        start_orientation (np.array): A quaternion describing the start orientation
                                      of the robot base
        default_endeffector (str, optional): The default endeffector used
                                             when controlling the robots position
    """

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 default_endeffector: str = None):

        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=True)

        self.number_of_joints = p.getNumJoints(self.urdf)
        self._joint_state_shape = self.get_joint_state()
        self._joint_name_to_index = {}
        self._link_name_to_index = {}
        kinematic_solver_map = []
        self._lower_joint_limit = np.zeros(self.number_of_joints)
        self._upper_joint_limit = np.zeros(self.number_of_joints)

        for joint_number in range(self.number_of_joints):
            joint_info = p.getJointInfo(self.urdf, joint_number)
            link_name = joint_info[12].decode("utf-8")
            self._link_name_to_index[link_name] = joint_number

            if joint_info[2] != 4:  # checks if the joint is not fixed
                joint_name = joint_info[1].decode("utf-8")
                self._joint_name_to_index[joint_name] = joint_number

                kinematic_solver_map.append(joint_number)

                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                if upper_limit < lower_limit:
                    lower_limit = -np.inf
                    upper_limit = np.inf
                self._lower_joint_limit[joint_number] = lower_limit
                self._upper_joint_limit[joint_number] = upper_limit

        self._kinematic_solver_map = np.array(kinematic_solver_map)

        if default_endeffector is None:
            last_link = max(self._link_name_to_index)
            self._default_endeffector_id = self._link_name_to_index[last_link]
        else:
            self._default_endeffector_id = self._convert_endeffector(
                default_endeffector)

        self.max_joint_force = 1000*np.ones(self.number_of_joints)
        for joint_number in range(self.number_of_joints):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

    def get_joint_limits(self, selected_joint_names: set = None):
        """Returns the lower and upper joint limits of the robot's moving joints.

        Args:
            selected_joint_names (set, optional): A set of joint names to retrieve
                                                limits for. Defaults to None,
                                                which retrieves all joint limits.

        Returns:
            Tuple[dict, dict]: Two dictionaries containing lower and upper joint
                            limits keyed by joint names.

        Raises:
            ValueError: If a joint name in `selected_joint_names` is not found in
                        the robot's joint index.
        """
        lower_joint_limit = {}
        upper_joint_limit = {}

        if selected_joint_names is None:
            # Retrieve limits for all joints
            for joint_name in self._joint_name_to_index:
                lower_joint_limit[joint_name] = self._lower_joint_limit[
                    self._joint_name_to_index[joint_name]
                ]
                upper_joint_limit[joint_name] = self._upper_joint_limit[
                    self._joint_name_to_index[joint_name]
                ]
        else:
            # Retrieve limits only for selected joints
            for joint_name in selected_joint_names:
                if joint_name in self._joint_name_to_index:
                    lower_joint_limit[joint_name] = self._lower_joint_limit[
                        self._joint_name_to_index[joint_name]
                    ]
                    upper_joint_limit[joint_name] = self._upper_joint_limit[
                        self._joint_name_to_index[joint_name]
                    ]
                else:
                    raise ValueError(
                        f"Joint name '{joint_name}' not found in the robot's "
                        f"joint index."
                    )

        return lower_joint_limit, upper_joint_limit

    def get_moveable_joints(self, selected_joint_names: set = None):
        """
        Retrieves the names and indices of the robot's moveable joints,
        sorted by their indices.

        Args:
            selected_joint_names (set, optional): A set of joint names to
                                                filter the results.
                                                If None, all moveable joints
                                                are included.

        Returns:
            Tuple[Tuple[str], Tuple[int]]:
                - A tuple of joint names (`joint_order`)
                - A tuple of corresponding joint indices (`joint_index`)
        """

        joint_items = self._joint_name_to_index.items()

        # Sort the joints by their indices
        sorted_joint_items = sorted(joint_items, key=lambda item: item[1])

        # Extract the joint names in the correct order
        joint_order = tuple(key for key, _ in sorted_joint_items)
        joint_index = tuple(self._joint_name_to_index[key] for key in joint_order)

        return joint_order, joint_index

    def get_joint_state(self):
        """Returns the position of each joint as a dictionary keyed with their name

        Returns:
             Dict[str,Dict[str,float]]: The state of all joinst

        """
        joint_state = {}
        for joint_number in range(self.number_of_joints):
            joint_info = p.getJointInfo(self.urdf, joint_number)
            if joint_info[2] != 4:  # checks if the joint is not fixed
                # convert byte string to string
                joint_name = joint_info[1].decode("utf-8")
                joint_state_list = p.getJointState(self.urdf, joint_number)

                single_joint_state = {'position': joint_state_list[0],
                                      'velocity': joint_state_list[1],
                                      'reaction force': joint_state_list[2],
                                      'torque': joint_state_list[3]}
                joint_state[joint_name] = single_joint_state
        return joint_state

    def set_joint_position(self, target: Dict[str,  float], ignore_limits=False):
        """Sets the target position for a number of joints.
           The maximum force of each joint is set according to the max_joint_force class attribute.

        Args:
            target (Dict[str,  float]): A dictionary containing the joint states to be set

        Raises:
            KeyError: If the specified joint state is not part of the Robot
        """
        if not all(key in self._joint_state_shape for key in target):
            raise KeyError('One or more joints are not part of the robot. ' +
                           'correct keys are: '+str(self._joint_state_shape.keys()))

        for joint, joint_position in target.items():
            joint_number = self._joint_name_to_index[joint]

            if ignore_limits is False:
                lower_joint_limit = self._lower_joint_limit[joint_number]
                upper_joint_limit = self._upper_joint_limit[joint_number]
                if joint_position > upper_joint_limit or joint_position < lower_joint_limit:
                    raise ValueError('The joint position '+str(joint_position) +
                                     ' is out of limit for joint '+joint+'. Its limits are:\n' +
                                     str(lower_joint_limit)+' and '+str(upper_joint_limit))

            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=joint_position)

    def reset_joint_position(self, target: Dict[str, float], ignore_limits=False):
        """Resets the robot's joints to specified positions.

        Args:
            target (Dict[str, float]): A dictionary mapping joint names to their
                                    desired positions.
            ignore_limits (bool, optional): If True, bypasses joint limit checks.
                                            Defaults to False.

        Raises:
            KeyError: If one or more joints in `target` are not part of the robot.
            ValueError: If a joint position in `target` is outside its limits and
                        `ignore_limits` is False.

        Returns:
            Dict[str, float]: The input target dictionary for reference.
        """
        # Ensure all specified joints exist in the robot's joint state
        if not all(key in self._joint_state_shape for key in target):
            raise KeyError(
                "One or more joints are not part of the robot. Correct keys are: "
                + str(self._joint_state_shape.keys())
            )

        # Iterate over target joint positions and reset each joint
        for joint_name, joint_position in target.items():
            if not ignore_limits:
                lower_joint_limit = self._lower_joint_limit[
                    self._joint_name_to_index[joint_name]
                ]
                upper_joint_limit = self._upper_joint_limit[
                    self._joint_name_to_index[joint_name]
                ]
                # Check if the joint position is within the defined limits
                if joint_position > upper_joint_limit or joint_position < lower_joint_limit:
                    raise ValueError(
                        f"The joint position {joint_position} is out of limits "
                        f"for joint '{joint_name}'. Its limits are: "
                        f"{lower_joint_limit} and {upper_joint_limit}."
                    )
            # Reset the joint state in the simulation
            p.resetJointState(
                self.urdf,
                self._joint_name_to_index[joint_name],
                targetValue=joint_position,
            )

        return target


    def get_endeffector_pose(self, endeffector_name: str = None):
        """Returns the position of the endeffector in world coordinates

        Args:
            endeffector (str, optional): The name of a different endeffector link

        Returns:
            np.array: The position of the endeffector
            np.array: The orientation of the endeffector as a quaternion
        """
        if endeffector_name is None:
            endeffector_id = self._default_endeffector_id
        else:
            endeffector_id = self._convert_endeffector(endeffector_name)

        link_state = p.getLinkState(self.urdf, endeffector_id)

        position = np.array(link_state[0])
        orientation = np.array(link_state[1])
        return position, orientation

    def set_endeffector_pose(self, target_position: np.array, target_orientation: np.array = None,
                             endeffector_name: str = None):
        """Sets the pose of a robots endeffector

        Args:
            target_position (np.array): The desired 3D position
            target_orientation (np.array, optional): The desired orientation as a quaternion.
                                                     Defaults to None.
            endeffector_name (str, optional): The name of a different endeffector.
                                              Defaults to None.
        """
        if endeffector_name is None:
            endeffector_id = self._default_endeffector_id
        else:
            endeffector_id = self._convert_endeffector(endeffector_name)

        if target_orientation is None:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                       endeffector_id,
                                                       target_position,
                                                       lowerLimits=self._lower_joint_limit,
                                                       upperLimits=self._upper_joint_limit)
        else:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                       endeffector_id,
                                                       target_position,
                                                       targetOrientation=target_orientation,
                                                       lowerLimits=self._lower_joint_limit,
                                                       upperLimits=self._upper_joint_limit)

        for index, joint_position in enumerate(joint_poses):
            joint_number = self._kinematic_solver_map[index]
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=joint_position)
        # for joint_index, joint_value in zip(self.get_moveable_joints()[1], joint_poses):
        #     p.resetJointState(
        #          self.urdf,
        #          joint_index,
        #          targetValue=joint_value)

    def reset_endeffector_pose(self, target_position: np.array, target_orientation: np.array = None,
                             endeffector_name: str = None):
        """Sets the pose of a robots endeffector

        Args:
            target_position (np.array): The desired 3D position
            target_orientation (np.array, optional): The desired orientation as a quaternion.
                                                     Defaults to None.
            endeffector_name (str, optional): The name of a different endeffector.
                                              Defaults to None.
        """
        if endeffector_name is None:
            endeffector_id = self._default_endeffector_id
        else:
            endeffector_id = self._convert_endeffector(endeffector_name)

        if target_orientation is None:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                       endeffector_id,
                                                       target_position,
                                                       lowerLimits=self._lower_joint_limit,
                                                       upperLimits=self._upper_joint_limit)
        else:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                       endeffector_id,
                                                       target_position,
                                                       targetOrientation=target_orientation,
                                                       lowerLimits=self._lower_joint_limit,
                                                       upperLimits=self._upper_joint_limit)

        for index, joint_position in enumerate(joint_poses):
            joint_number = self._kinematic_solver_map[index]
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=joint_position)
        for joint_index, joint_value in zip(self.get_moveable_joints()[1], joint_poses):
            p.resetJointState(
                 self.urdf,
                 joint_index,
                 targetValue=joint_value)

    def reset_robot(self, start_position: np.array, start_orientation: np.array,
                    joint_values: list = None):
        """resets the robots joints to 0 and the base to a specified position and orientation

        Args:
            start_position (np.array): a 3 dimensional position
            start_orientation (np.array): a 4 dimensional quaternion representing
                                          the desired orientation
            joint_values (list): Allows to reset the joint state of the robot given
                                 a list of positions.
                                 Defaults to None in which case the joints remain in their current
                                 configuration.
        """
        self.set_world_state(start_position, start_orientation)

        if joint_values is None:
            joint_values = np.zeros(self.number_of_joints)
        for joint in range(self.number_of_joints):
            p.resetJointState(self.urdf, joint,
                              targetValue=joint_values[joint])

    def set_world_state(self, start_position: np.array, start_orientation: np.array):
        """Resets the robots base to a specified position and orientation

        Args:
            start_position (np.array): a 3 dimensional position
            start_orientation (np.array): a 4 dimensional quaternion representing
                                          the desired orientation
        """
        p.resetBasePositionAndOrientation(
            self.urdf, start_position, start_orientation)

    def get_world_state(self):
        """Returns the position and orientation of the robot relative to the world

        Returns:
            list: the 3 dimensional position vector of the robot base
            list: a 4 dimensional quaternion representing the orientation of the robot base
        """
        return p.getBasePositionAndOrientation(self.urdf)

    def _convert_endeffector(self, endeffector: str):
        """Internal Function which converts an endeffector name to an id

        Args:
            endeffector (str): The name of the endeffector link

        Raises:
            TypeError: if the name is not a string.
            ValueError: if the endeffector name is not valid

        Returns:
            int: The corresponding link index to the endeffector id.
        """
        if not isinstance(endeffector, str):
            raise TypeError(
                "The Endeffector must be a String describing a URDF link")
        if not endeffector in self._link_name_to_index:
            raise ValueError("Invalid Endeffecot name! valid names are: " +
                             str(self._link_name_to_index.keys()))

        return self._link_name_to_index[endeffector]
