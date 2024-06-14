"""A simple class that encapsulates mobile robots moving on the x-y plane in pybullet."""
import numpy as np
import pybullet as p

#Template Class for all AGV Robots
class AGVRobot:

    def __init__(self,urdf_model: str, start_position: np.array, start_orientation: np.array,
                 position_controller=None):
        """Template class for all AGV Robots

        Args:
            urdf_model (str): The path to the URDF file of the robot.
            start_position (np.array): The start position of the robot.
            start_orientation (np.array): The start orientation of the robot.
            position_controller (function, optional): A function that calculates the wheel commands
                                                      for a given distance and angle to the target
                                                      as well as the target angle error. The function
                                                      should return a 2 dimensional vector containing
                                                      the linear and angular velocity of the robot.
                                                      Defaults to None. In this case a standard
                                                      position controller is used.

        """

        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        if position_controller is None:
            self.position_controller = self.standard_position_controller
        else:
            self.position_controller = position_controller




    def _calculate_position_error(self):
        """Calculates the position error of the robot in polar coordinates.

        Returns:
            float: The distance to the target position.
            float: The angle to the target position from the robot front.
        """
        current_position, current_orientation = p.getBasePositionAndOrientation(self.urdf)
        current_orientation = np.array(p.getEulerFromQuaternion(current_orientation))


        #calculate the angle between the front of the robot and the vector pointing to the target
        angle_to_target = np.arctan2(self.target_pose[1] - current_position[1],
                           self.target_pose[0] - current_position[0]) - current_orientation[2]

        #map angle onto the interval [-pi, pi]
        if angle_to_target > np.pi:
            angle_to_target -= 2*np.pi
        elif angle_to_target < -np.pi:
            angle_to_target += 2*np.pi

        #calculate the distance between the current position and the target position
        distance_to_target = np.linalg.norm(np.array(self.target_pose[:2]) -
                                            np.array(current_position[:2]))

        target_angle_error = current_orientation[2] - self.target_pose[2]

        #map angle onto the interval [-pi, pi]
        if target_angle_error > np.pi:
            target_angle_error -= 2*np.pi
        elif target_angle_error < -np.pi:
            target_angle_error += 2*np.pi

        return distance_to_target, angle_to_target, target_angle_error

    def standard_position_controller(self,distance,angle,target_angle_error):
        raise NotImplementedError("The standard position controller has to be implemented in a subclass.")


    def set_velocity(self,velocity_vector):
        raise NotImplementedError("The set_velocity method has to be implemented in a subclass.")

    def update_position_loop(self):
        """Updates the position of the robot in a loop.
        """

        distance, angle, target_angle_error = self._calculate_position_error()
        velocity_vector = self.position_controller(distance,angle,target_angle_error)
        self.set_velocity(velocity_vector)

    def set_target_pose(self, target_position: np.array,target_orientation: np.array):
        """Sets the target posisition of the robot and executes as velocity command to reach it.

        Args:
            target_position (np.array): The target position of the robot.
                                        Note that while the target is 3 dimensional,
                                        only the x and y coordinates are used.
            target_orientation (np.array): The target orientation of the robot as a quaternion.
        """

        yaw_target_orientation = np.array(p.getEulerFromQuaternion(target_orientation))[2]
        self.target_pose = np.append(target_position[:2],yaw_target_orientation)

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



class DiffDriveAGV(AGVRobot):


    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 left_wheel_name: str, right_wheel_name: str,diff_drive_params: dict,
                 position_controller=None):
        """Differential Drive based AGV Robot

        Args:
            urdf_model (str): The path to the URDF file of the robot.
            start_position (np.array): The start position of the robot.
            start_orientation (np.array): The start orientation of the robot.
            left_wheel_name (str): The name of the left wheel joint.
            right_wheel_name (str): The name of the right wheel joint.
            diff_drive_params (dict): A dictionary containing the parameters of the robot.
            position_controller (function, optional): A function that calculates the wheel commands
                                                      for a given distance and angle to the target
                                                      as well as the target angle error. The function
                                                      should return a 2 dimensional vector containing
                                                      the linear and angular velocity of the robot.
                                                      Defaults to None. In this case a standard
                                                      position controller is used.
        """

        super().__init__(urdf_model, start_position, start_orientation, position_controller)

        num_joints = p.getNumJoints(self.urdf)
        self.left_wheel_index = None
        self.right_wheel_index = None

        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.urdf, joint_index)
            joint_name = joint_info[1].decode('UTF-8')

            if joint_name == left_wheel_name:
                self.left_wheel_index = joint_index
            elif joint_name == right_wheel_name:
                self.right_wheel_index = joint_index

        if self.left_wheel_index is None or self.right_wheel_index is None:
            raise ValueError("Wheel names not found in URDF.")


        self.target_pose = np.zeros(3)

        self.wheel_radius = diff_drive_params["wheel_radius"]
        self.track_width = diff_drive_params["track_width"]
        self.max_linear_velocity = diff_drive_params["max_linear_velocity"]
        self.max_angular_velocity = diff_drive_params["max_angular_velocity"]


        # set the control mode of the wheels to VELOCITY_CONTROL
        p.setJointMotorControl2(self.urdf, self.left_wheel_index, p.VELOCITY_CONTROL)
        p.setJointMotorControl2(self.urdf, self.right_wheel_index, p.VELOCITY_CONTROL)

    def _calculate_wheel_comands(self, velocity_vector):
        """Calculates the wheel commands for a differential drive robot.

        Args:
            linear_velocity (float): The linear velocity of the robot.
            angular_velocity (float): The angular velocity of the robot.

        Returns:
            np.array: The wheel commands.
        """
        linear_velocity = velocity_vector[0]
        angular_velocity = velocity_vector[1]

        turning_contribution = angular_velocity * self.track_width / (2*self.wheel_radius)
        driving_contribution = linear_velocity / self.wheel_radius
        left_wheel_velocity = driving_contribution - turning_contribution
        right_wheel_velocity = driving_contribution + turning_contribution


        return np.array([left_wheel_velocity, right_wheel_velocity])

    def set_velocity(self, velocity_vector):
        """Sets the velocity of the robot.

        Args:
            velocity_vector (np.array): The velocity vector of the robot.
                                        In this case the vector contains the linear
                                        and angular velocity of the robot.
        """
        linear_velocity = velocity_vector[0]
        angular_velocity = velocity_vector[1]

        #clip the velocities to the maximum velocities
        linear_velocity = np.clip(linear_velocity,
                                    -self.max_linear_velocity,
                                    self.max_linear_velocity)
        angular_velocity = np.clip(angular_velocity,
                                    -self.max_angular_velocity,
                                    self.max_angular_velocity)

        wheel_commands = self._calculate_wheel_comands([linear_velocity, angular_velocity])
        p.setJointMotorControl2(self.urdf,
                                self.left_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[0])
        p.setJointMotorControl2(self.urdf,
                                self.right_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[1])

    def standard_position_controller(self,distance,angle,target_angle_error):
        """ A simple position controller for a differential drive robot.


        Args:
            distance (float): the distance to the target position
            angle (float): the angle to the target position in radians from -pi to pi
            target_angle_error (float): the angle error between the robot orientation and the target orientation
                                         in radians from -pi to pi

        Returns:
            List: A velocity vector containing the linear and angular velocity of the robot.
        """
        kp_lin=1
        kp_ang= 0.8
        kp_target_ang = 0.4

        distance_precision = 0.05
        angle_precision = 0.01

        linear_velocity = np.clip(kp_lin*distance,
                                  -self.max_linear_velocity,
                                  self.max_linear_velocity)
        # scale linear velocity so that it it becomes smaller the larger the angle is
        linear_velocity *= np.clip(1 - 2*np.abs(angle)/np.pi, -1, 1)

        angular_velocity = 1*kp_ang*angle
        angular_velocity +=  1*kp_target_ang*target_angle_error
        angular_velocity = np.clip(angular_velocity,
                                      -self.max_angular_velocity,
                                      self.max_angular_velocity)

        if (distance < distance_precision) and (np.abs(target_angle_error) < angle_precision):
            linear_velocity = 0
            angular_velocity = 0


        return [linear_velocity, angular_velocity]