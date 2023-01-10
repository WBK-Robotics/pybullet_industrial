from ctypes.wintypes import POINT
from time import sleep
from gcode_class import *
import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np

def move_along_path(endeffector: pi.EndeffectorTool, path: pi.ToolPath, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (pi.EndeffectorTool): Endeffector to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop at the end of the movement.
    """
    for positions, orientations, tool_path in path:
        endeffector.set_tool_pose(positions, orientations)
        for _ in range(10):
            p.stepSimulation()
    if stop:
        for _ in range(100):
            p.stepSimulation()
    
def move_robot(robot: pi.RobotBase, path: pi.ToolPath, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (pi.EndeffectorTool): Endeffector to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop at the end of the movement.
    """
    for positions, orientations, tool_path in path:
        robot.set_endeffector_pose(positions, orientations)
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.005)
    if stop:
        for _ in range(100):
            p.stepSimulation()        


if __name__ == "__main__":
    
    #os.system("'py -m pip install src\'")

    # Erstellt die Simulationsumgebung
    
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'gripper_cad.urdf')
    urdf_file3 = os.path.join(dirname,
                              'robot_descriptions', 'gripper_cylinder.urdf')
    urdf_file4 = os.path.join(dirname,
                              'robot_descriptions', 'gripper.urdf')
    urdf_files = [urdf_file2, urdf_file3, urdf_file4]
                              
    #urdf_file3 = os.path.join(dirname,
    #                          'robot_descriptions', 'cube_small.urdf')


    physics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                              '--background_color_green=1 ' +
                                              '--background_color_blue=1')
    p.setPhysicsEngineParameter(numSolverIterations=10000)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=10000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -10)
    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

    start_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    gripper = []
    i = 0
    shift = 0
    # for n in urdf_files:
    #     gripper.append(pi.Gripper(
    #             n, [2.7, -0.5+ shift, 1.2], start_orientation))
    #     i = i + 1
    #     shift = shift + 0.2
    
    gripper_location  = []
    
    
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    
    robot.set_joint_position(
        {'q2': np.deg2rad(-15.0), 'q3': np.deg2rad(-90.0)})
    for _ in range(100):
        p.stepSimulation()

    dirname = os.path.dirname(__file__)
    textfile = os.path.join(dirname,'code.txt')
    gcode = Gcode_class(textfile,robot)
   
   