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
        left_wheel_velocity = (2 * linear_velocity - angular_velocity * self.track_width) / (2 * self.wheel_radius)
        right_wheel_velocity = (2 * linear_velocity + angular_velocity * self.track_width) / (2 * self.wheel_radius)

        return np.array([left_wheel_velocity, right_wheel_velocity])

    def set_velocity(self, linear_velocity: float, angular_velocity: float):
        """Sets the velocity of the robot.
        
        Args:
            linear_velocity (float): The linear velocity of the robot.
            angular_velocity (float): The angular velocity of the robot.
        """
        wheel_commands = self.__calculate_wheel_comands(linear_velocity, angular_velocity)
        p.setJointMotorControl2(self.urdf, self.left_wheel_index, p.VELOCITY_CONTROL, targetVelocity=wheel_commands[0])
        p.setJointMotorControl2(self.urdf, self.right_wheel_index, p.VELOCITY_CONTROL, targetVelocity=wheel_commands[1])


    def set_target_position(self, target_position: np.array):
        """Sets the target pose of the robot.
        
        Args:
            target_position (np.array): The target position of the robot.
            target_orientation (np.array): The target orientation of the robot.
        """
        
        #a simple controller that turns the robot towards the target before moving forward

        current_position, current_orientation = p.getBasePositionAndOrientation(self.urdf)
        current_orientation = np.array(p.getEulerFromQuaternion(current_orientation))


        #calculate the angle between the front of the robot and the vector pointing to the target
        angle = np.arctan2(target_position[1] - current_position[1], target_position[0] - current_position[0]) - current_orientation[2]
        
        #map angle onto the interval [-pi, pi]
        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi

        #calculate the distance between the current position and the target position
        distance = np.linalg.norm(np.array(target_position) - np.array(current_position))
        print(distance,angle)
        # linear velocity is proportional to the distance smoothed by sigmoid function 
        # to be within the range of -max_linear_velocity and max_linear_velocity
        kp_lin=0.4
        kp_ang=0.8

        linear_velocity = np.clip(kp_lin*distance, -self.max_linear_velocity, self.max_linear_velocity)

        # scale linear velocity so that it it becomes smaller the larger the angle is
        linear_velocity *= np.clip(1 - 2*np.abs(angle)/np.pi, 0, 1)
 

        # angular velocity is proportional to the angle smoothed by sigmoid function
        # to be within the range of -max_angular_velocity and max_angular_velocity
        angular_velocity = -1*np.clip(kp_ang*angle, -self.max_angular_velocity, self.max_angular_velocity)

        # scala angular velocity so that it becomes smaller the smaller the distance is to avoid singularity
        angular_velocity *= np.clip(10*distance, 0, 1)


        self.set_velocity(linear_velocity, angular_velocity)



        



if __name__ == "__main__":
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
                         "max_linear_velocity": 1,
                         "max_angular_velocity": 0.5}
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                              'robot_descriptions', 'diff_drive_agv.urdf')
    agv = DiffDriveAGV(urdf_file, [0, 0, 0.3], [0, 0, 0, 1], "left_wheel_joint", "right_wheel_joint", diff_drive_params)

    print(agv.track_width, agv.wheel_radius)
    # correct wheel radius is 0.17775

    while True:
        p.stepSimulation()
        agv.set_target_position([0, 4, 0])

        

