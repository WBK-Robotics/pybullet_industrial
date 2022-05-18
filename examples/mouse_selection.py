import os
import time
import pybullet as p
import wbk_sim as wbk
import numpy as np


def get_object_id_from_mouse():
    mouseEvents = p.getMouseEvents()
    for e in mouseEvents:
        if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
            mouseX = e[1]
            mouseY = e[2]
            width, height, _, _, _, camera_forward, horizontal, vertical, _, _, dist, camera_target = p.getDebugVisualizerCamera()
            horizontal = np.array(horizontal)
            vertical = np.array(vertical)
            camera_target = np.array(camera_target)
            camera_forward = np.array(camera_forward)

            camera_position = camera_target-dist*camera_forward #current position of the debug camera
            ray_start_pos = camera_position
            far_plane = 10000
            ray_forward = camera_target-camera_position
            ray_forward = far_plane*ray_forward/ np.linalg.norm(ray_forward)

            dHor = horizontal/width
            dVer = vertical/height

            ray_end_pos = ray_start_pos+ ray_forward - 0.5*horizontal+0.5*vertical+float(mouseX)* dHor - float(mouseY)*dVer
            rayInfo = p.rayTest(ray_start_pos, ray_end_pos )
            hit = rayInfo[0]
            return hit[0], hit[1]
    return -1,-1



if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(
        dirname, 'robot_descriptions', 'comau_NJ290_3-0_m.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation, 'link6')
    robot = wbk.RobotBase(urdf_file1, [1, 0, 0], start_orientation, 'link6')

    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    drawing_position = [2.4, 0, 1.2]
    wbk.draw_point(target_position, length=0.05)

    colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
    currentColor = 0

    p.getCameraImage(64, 64, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    while (1):
        objectUid, object_index = get_object_id_from_mouse()
        if (objectUid >= 0):
            p.changeVisualShape(objectUid, object_index, rgbaColor=colors[currentColor])
            currentColor += 1
            if (currentColor >= len(colors)):
                currentColor = 0

