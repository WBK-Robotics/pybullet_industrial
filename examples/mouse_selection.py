import os
import time
import pybullet as p
import wbk_sim as wbk
import numpy as np


def getRayFromTo(mouseX, mouseY):
  width, height, _, _, _, camera_forward, horizon, vertical, _, _, dist, camera_target = p.getDebugVisualizerCamera(
  )
  camera_position = [
      camera_target[0] - dist * camera_forward[0], camera_target[1] - dist * camera_forward[1],
      camera_target[2] - dist * camera_forward[2]
  ]
  far_plane = 10000
  ray_forward = [(camera_target[0] - camera_position[0]), (camera_target[1] - camera_position[1]), (camera_target[2] - camera_position[2])]
  invLen = far_plane * 1. / (np.sqrt(ray_forward[0] * ray_forward[0] + ray_forward[1] *
                                      ray_forward[1] + ray_forward[2] * ray_forward[2]))
  ray_forward = [invLen * ray_forward[0], invLen * ray_forward[1], invLen * ray_forward[2]]
  ray_start_pos = camera_position
  oneOverWidth = float(1) / float(width)
  oneOverHeight = float(1) / float(height)
  dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
  dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]

  ray_end_pos = [
      ray_start_pos[0] + ray_forward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
      float(mouseY) * dVer[0], ray_start_pos[1] + ray_forward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
      float(mouseX) * dHor[1] - float(mouseY) * dVer[1], ray_start_pos[2] + ray_forward[2] -
      0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
  ]
  return ray_start_pos, ray_end_pos

def get_object_id_from_mouse():
    mouseEvents = p.getMouseEvents()
    for e in mouseEvents:
        if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
            mouseX = e[1]
            mouseY = e[2]
            rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
            rayInfo = p.rayTest(rayFrom, rayTo)
            #p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
            hit = rayInfo[0]
        return hit[0], hit[1]



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

        mouseEvents = p.getMouseEvents()
        for e in mouseEvents:
            if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
                mouseX = e[1]
                mouseY = e[2]
                rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
                rayInfo = p.rayTest(rayFrom, rayTo)
                #p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
                print(rayInfo)    
                hit = rayInfo[0]
                objectUid = hit[0]
                #print(hit)
                if (objectUid >= 0):
                    #p.removeBody(objectUid)
                    p.changeVisualShape(objectUid, hit[1], rgbaColor=colors[currentColor])
                    currentColor += 1
                    if (currentColor >= len(colors)):
                        currentColor = 0

