import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from path_planner import PathPlanner
from robot_base import RobotBase
from prm_planner import PRMPlanner
from g_code_processor import GCodeProcessor

def seting_up_enviroment():
       # Setting up the paths
    working_dir = os.path.dirname(__file__)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    urdf_robot = os.path.join(
        working_dir, '..', '..', 'robot_descriptions', 'comau_nj290_robot.urdf')

    urdf_fofa = os.path.join(
        working_dir,'..', '..', 'Objects', 'FoFa', 'FoFa.urdf')

    # Setting up the simulation
    start_pos = np.array([2.0, -6.5, 0])
    p.connect(p.GUI, options='--background_color_red=1 ' +
              '--background_color_green=1 ' +
              '--background_color_blue=1')
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0, cameraYaw=50.0,
        cameraPitch=-30,
        cameraTargetPosition=np.array([1.9, 0, 1]) + start_pos)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setPhysicsEngineParameter(numSolverIterations=5000, enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF(urdf_fofa, useFixedBase=True, globalScaling=0.001)
    return urdf_robot, start_pos, start_orientation

if __name__ == "__main__":

    urdf_robot, start_pos, start_orientation = seting_up_enviroment()

    # Setting up robot position 
    robot = RobotBase(urdf_robot, start_pos, start_orientation)
    #Setting up PRMPlanner
    path_planner = PRMPlanner(robot, num_samples= 1000, num_neighbors= 10)
    
    start_config = np.array([0,0,0,0,0,0])
    end_config = np.array([1,1,1,1,1,1])
    robot.set_robot(start_config) 

    # Setting a valid configuration & ignoring "valid" collisions
    path_planner.ignored_collisions = path_planner.detect_collision()

    # Creation of the free C-space
    path_planner.create_prm()

    # Creation of ConfigPath
    config_path = path_planner.create_collision_free_path(start_config, end_config)

    # Run ConfigPath
    g_code_object = GCodeProcessor()
    g_code_object.g_code = g_code_object.path_to_g_code(config_path)

    g_code_object = iter(g_code_object)

    # Iterate over the demonstration object
    for _ in g_code_object:
    # Execute the simulation steps
        for _ in range(200):
            p.stepSimulation()
