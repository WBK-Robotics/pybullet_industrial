"""This is an example for an AGV following a set path using a custom controller."""
import pybullet as p
import os


import pybullet_industrial as pi
import pybullet_data
import time
import os

def trajectory_follower_controller(distance,angle,target_angle_error):
    """ A simple trajectory following controller that neglects the target_angle_error and simply
        orients the robot towards the current target position.
        The controller is a simple proportional controller that
        takes the distance and angle to the target
        and returns a linear and angular velocity command to the robot.
        """
    kp_lin=1
    kp_ang=1

    linear_velocity = kp_lin*distance,
    angular_velocity = -1*kp_ang*angle,

    return [linear_velocity[0], angular_velocity[0]]

if __name__ == "__main__":

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

    agv = pi.DiffDriveAGV(urdf_file, [0, 0, 0.3], [0, 0, 0, 1],
                       "left_wheel_joint",
                       "right_wheel_joint",
                       diff_drive_params,
                       position_controller=trajectory_follower_controller)

    agv.set_world_state([2.25,0,1], [0,0,0,1])


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



