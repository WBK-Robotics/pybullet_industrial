"""A simple class that encapsulates mobile robots."""
import numpy as np
import pybullet as p


class DiffDriveAGV:


    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 left_wheel_name: str, right_wheel_name: str,diff_drive_params: dict):

        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

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







    def __calculate_wheel_comands(self, linear_velocity: float, angular_velocity: float):
        """Calculates the wheel commands for a differential drive robot.

        Args:
            linear_velocity (float): The linear velocity of the robot.
            angular_velocity (float): The angular velocity of the robot.

        Returns:
            np.array: The wheel commands.
        """
        turning_contribution = angular_velocity * self.track_width / (2*self.wheel_radius)
        driving_contribution = linear_velocity / self.wheel_radius
        left_wheel_velocity = driving_contribution - turning_contribution
        right_wheel_velocity = driving_contribution + turning_contribution


        return np.array([left_wheel_velocity, right_wheel_velocity])

    def set_velocity(self, linear_velocity: float, angular_velocity: float):
        """Sets the velocity of the robot.

        Args:
            linear_velocity (float): The linear velocity of the robot.
            angular_velocity (float): The angular velocity of the robot.
        """
        wheel_commands = self.__calculate_wheel_comands(linear_velocity, angular_velocity)
        p.setJointMotorControl2(self.urdf,
                                self.left_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[0])
        p.setJointMotorControl2(self.urdf,
                                self.right_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[1])

    def __calculate_position_error(self):
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
        distance_to_target = np.linalg.norm(np.array(self.target_pose) - np.array(current_position))

        target_angle_error = current_orientation[2] - self.target_pose[2]

        return distance_to_target, angle_to_target, target_angle_error


    def position_controller(self,distance,angle,target_angle_error):
        kp_lin=0.4
        kp_ang=0.8

        linear_velocity = np.clip(kp_lin*distance,
                                  -self.max_linear_velocity,
                                  self.max_linear_velocity)
        # scale linear velocity so that it it becomes smaller the larger the angle is
        linear_velocity *= np.clip(1 - np.abs(angle)/np.pi, 0, 1)


        angular_velocity = -1*np.clip(kp_ang*angle,
                                      -self.max_angular_velocity,
                                      self.max_angular_velocity)
        # scala angular velocity so that it becomes smaller
        # with decreasing distance to avoid singularity
        angular_velocity *= np.clip(10*(distance-0.1), 0, 1)

        return linear_velocity, angular_velocity

    def update_position_loop(self):

        distance, angle, target_angle_error = self.__calculate_position_error()

        linear_velocity, angular_velocity = self.position_controller(distance,angle,target_angle_error)

        self.set_velocity(linear_velocity, angular_velocity)

    def set_target_position(self, target_position: np.array):
        """Sets the target posisition of the robot and executes as velocity command to reach it.

        Args:
            target_position (np.array): The target position of the robot.
                                        Note that while the target is 3 dimensional,
                                        only the x and y coordinates are used.
        """

        self.target_pose[:2] = target_position[:2]




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





if __name__ == "__main__":
    import pybullet_industrial as pi
    import pybullet_data
    import time
    import os


    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                                '--background_color_green=1 ' +
                                                '--background_color_blue=1')

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -10)

    diff_drive_params = {"wheel_radius": 0.2,
                         "track_width": 0.3,
                         "max_linear_velocity": 2,
                         "max_angular_velocity": 1}
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                              'robot_descriptions', 'diff_drive_agv.urdf')

    agv = DiffDriveAGV(urdf_file, [0, 0, 0.3], [0, 0, 0, 1], "left_wheel_joint", "right_wheel_joint", diff_drive_params)

    agv.set_world_state([2.25,0,1], [0,0,0,1])
    print(agv.track_width, agv.wheel_radius)

    test_path = pi.build_box_path(
        [0,0,0], [4.5, 6.6], 0.8, [0, 0, 0, 1], 200)

    test_path.draw()
    while True:
        for positions, orientations, _ in test_path:
            agv.set_target_position(positions)
            for _ in range(200):
                agv.update_position_loop()
                p.stepSimulation()
            time.sleep(0.01)



