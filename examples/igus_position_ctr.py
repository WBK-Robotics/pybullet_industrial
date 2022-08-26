import os
import time
import pybullet as p
import pybullet_industrial as pi
from lemniscate import build_lemniscate_path
import numpy as np

"""Example of a igus robot following a lemniscate path 
   with the current GUI view saved as a mp4 file.
   This functionality requires ffmpeg to be installed. Otherwise no video is saved."""
if __name__ == "__main__":

    # connect to the GUI and set the background color to white
    physics_client = p.connect(
        p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # disables gui and grid

    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'igus_4DOF_SV.urdf')
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])

    # color the robot
    for i in range(4):
        p.changeVisualShape(
            robot.urdf, -1+i, rgbaColor=[20/255, 68/255, 102/255, 1])
    p.changeVisualShape(
        robot.urdf, 3, rgbaColor=[238/255, 183/255, 13/255, 1])

    p.setRealTimeSimulation(1)

    target_position = np.array([0, 0.3])
    test_path = build_lemniscate_path(target_position, 400, 0, 0.3)

    pi.draw_path(test_path, width=4, color=[0/255, 150/255, 130/255])
    # this function requires ffmpeg to be installed
    video_log = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "igus.mp4")
    for i in range(400):
        robot.set_endeffector_pose(
            test_path[:, i], endeffector_name='link4')
        time.sleep(0.005)
    p.stopStateLogging(video_log)
    p.disconnect()
