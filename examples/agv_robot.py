"""A simple class that encapsulates mobile robots."""
import numpy as np
import pybullet as p


class DiffDriveAGV:


    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 left_wheel_name: str, right_wheel_name: str,diff_drive_params: dict = None):
        
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
        
        
        if diff_drive_params is None:
            self.__estimate_diff_drive_params(left_wheel_name, right_wheel_name)
        else:
            self.wheel_radius = diff_drive_params["wheel_radius"]
            self.track_width = diff_drive_params["track_width"]
        

        # set the control mode of the wheels to VELOCITY_CONTROL
        p.setJointMotorControl2(self.urdf, self.left_wheel_index, p.VELOCITY_CONTROL)
        p.setJointMotorControl2(self.urdf, self.right_wheel_index, p.VELOCITY_CONTROL)


    def __estimate_diff_drive_params(self,left_wheel_name,right_wheel_name):
        """Tries to estimate the diff drive parameters of the robot.
           Often not very accurate and requires manual tuning.
        
        Returns:
            np.array: The kinematic dimensions of the robot.
        """

        # Extract kinematic parameters
        # Assuming the wheel radius is the same for both wheels
        left_wheel_info = p.getJointInfo(self.urdf, self.left_wheel_index)
        right_wheel_info = p.getJointInfo(self.urdf, self.right_wheel_index)
        wheel_radius = left_wheel_info[8]  # This is a simplification, usually requires more detailed extraction

        # To calculate the track width, we need the positions of the wheels
        # This is a simplification and assumes the wheels are directly opposite each other
        left_wheel_position = p.getLinkState(self.urdf, self.left_wheel_index)[0]
        right_wheel_position = p.getLinkState(self.urdf, self.right_wheel_index)[0]
        track_width = abs(left_wheel_position[1] - right_wheel_position[1])  # Assuming Y-axis separation

        self.wheel_radius = wheel_radius
        self.track_width = track_width


        
    
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



if __name__ == "__main__":
    import pybullet_data
    import time
    

    pysics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                                '--background_color_green=1 ' +
                                                '--background_color_blue=1')
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -10)

    diff_drive_params = {"wheel_radius": 0.17775, "track_width": 0.5}
    agv = DiffDriveAGV("husky/husky.urdf", [0, 0, 0], [0, 0, 0, 1], "rear_left_wheel", "rear_right_wheel", diff_drive_params)

    print(agv.track_width, agv.wheel_radius)
    # correct wheel radius is 0.17775

    while True:
        p.stepSimulation()
        agv.set_velocity(0.0, 2.4)
        time.sleep(0.02)
        

