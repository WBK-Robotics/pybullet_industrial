"""A simple class that encapsulates mobile robots."""
import numpy as np
import pybullet as p



#Template Class for all AGV Robots
class AGVRobot:

    def __init__(self,urdf_model: str, start_position: np.array, start_orientation: np.array,
                 position_controller=None):

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
        pass

    def set_velocity(self,velocity_vector):
        pass

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
        """Initializes the robot.

        Args:
            urdf_model (str): The path to the URDF file of the robot.
            start_position (np.array): The start position of the robot.
            start_orientation (np.array): The start orientation of the robot.
            left_wheel_name (str): The name of the left wheel joint.
            right_wheel_name (str): The name of the right wheel joint.
            diff_drive_params (dict): A dictionary containing the parameters of the robot.
            position_controller (function, optional): A function that calculates the wheel commands
                                                      for a given distance and angle to the target.
                                                      Defaults to None. In this case the standard
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

    def set_velocity(self, velocity_vector):
        """Sets the velocity of the robot.

        Args:
            velocity_vector (np.array): The velocity vector of the robot.
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

        wheel_commands = self.__calculate_wheel_comands(linear_velocity, angular_velocity)
        p.setJointMotorControl2(self.urdf,
                                self.left_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[0])
        p.setJointMotorControl2(self.urdf,
                                self.right_wheel_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheel_commands[1])


    def standard_position_controller(self,distance,angle,target_angle_error):
        print(distance,angle,target_angle_error)
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

        angular_velocity = -1*kp_ang*angle
        # scala angular velocity so that it becomes smaller
        # with decreasing distance to avoid singularity
        #angular_velocity *= np.clip(10*(distance-1), 0, 1)

        # add contribution based on target_angle_error
        angular_velocity +=  -1*kp_target_ang*target_angle_error
        angular_velocity = np.clip(angular_velocity,
                                      -self.max_angular_velocity,
                                      self.max_angular_velocity)

        if (distance < distance_precision) and (np.abs(target_angle_error) < angle_precision):
            linear_velocity = 0
            angular_velocity = 0

        return [linear_velocity, angular_velocity]







if __name__ == "__main__":
    import pybullet_industrial as pi
    import pybullet_data
    import time
    import os

    def trajectory_follower_controller(distance,angle,target_angle_error):
        kp_lin=1
        kp_ang=1

        linear_velocity = kp_lin*distance,
        angular_velocity = -1*kp_ang*angle,

        return [linear_velocity[0], angular_velocity[0]]


    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                                '--background_color_green=1 ' +
                                                '--background_color_blue=1')

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -10)

    diff_drive_params = {"wheel_radius": 0.2,
                         "track_width": 0.3,
                         "max_linear_velocity": 0.8,
                         "max_angular_velocity": 0.8}
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                              'robot_descriptions', 'diff_drive_agv.urdf')

    agv = DiffDriveAGV(urdf_file, [0, 0, 0.3], [0, 0, 0, 1],
                       "left_wheel_joint",
                       "right_wheel_joint",
                       diff_drive_params,
                       position_controller=trajectory_follower_controller)

    agv.set_world_state([2.25,0,1], [0,0,0,1])
    print(agv.track_width, agv.wheel_radius)

    test_path = pi.build_box_path(
        [0,0,0], [4.5, 6.6], 0.8, [0, 0, 0, 1], 200)

    test_path.draw()

    #spawn a sphere mulitbody that highlights the target position
    sphere_visual = p.createVisualShape(p.GEOM_SPHERE,
                                        radius=0.1,
                                        rgbaColor=[1, 0, 0, 1])

    sphere = p.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=sphere_visual,
                                basePosition=[0, 0, 0.1])

    while True:
        positions,orientations = [0,0,0.1],[0,0,0,1]
        for positions, orientations, _ in test_path:
            p.resetBasePositionAndOrientation(sphere, positions, orientations)
            agv.set_target_pose(positions,orientations)
            for _ in range(50):
                agv.update_position_loop()
                p.stepSimulation()

            actual_pos, actual_ori = agv.get_world_state()

            time.sleep(0.01)



