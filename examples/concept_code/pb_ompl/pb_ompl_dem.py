import os
import pybullet as p
import pybullet_data
import numpy as np
import pybullet_industrial as pi
from robot_base import RobotBase
from pb_ompl import PbOMPL
import utils
from itertools import product

# from g_code_processor import GCodeProcessor

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

def clear_obstacles(obstacles):
    for obstacle in obstacles:
        p.removeBody(obstacle)

def add_box(box_pos, half_box_size):
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

    return box_id


if __name__ == "__main__":

    urdf_robot, start_pos, start_orientation = seting_up_enviroment()

    # Setting up robot position 
    robot = RobotBase(urdf_robot, start_pos, start_orientation)
    #Setting up PRMPlanner

    # setup pb_ompl
    obstacles = []
    pb_ompl_interface = PbOMPL(robot,obstacles)
    obstacle = add_box([1, 0, 0.7], [0.5, 0.5, 0.05])
    obstacles.append(obstacle)
    pb_ompl_interface.set_obstacles(obstacles)
    pb_ompl_interface.set_planner("BITstar")

    start = [0,0,0,-1,0,1.5]
    goal = [0, 1, -0.5, -0.1, 0, 0.2]

    robot.set_robot(start)

    # Testing collision controll
    
    # Configuration for collision checking
    self_collisions = True
    allow_collision_links = []


    check_link_pairs = utils.get_self_link_pairs(robot.urdf, robot.joint_idx) if self_collisions else []
    moving_links = frozenset(
        [item for item in utils.get_moving_links(robot.urdf, robot.joint_idx) if not item in allow_collision_links])
    moving_bodies = [(robot.urdf, moving_links)]
    check_body_pairs = list(product(moving_bodies, obstacles))

    # check self-collision
    robot.set_robot(pb_ompl_interface.state_to_list(robot.state))
    for link1, link2 in check_link_pairs:
        if utils.pairwise_link_collision(robot.urdf, link1, robot.urdf, link2):
            allow_collision_links.append((link1, link2))
            # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))

    # check collision against environment
    for body1, body2 in check_body_pairs:
        if utils.pairwise_collision(body1, body2):
            # print('body collision', body1, body2)
            # print(get_body_name(body1), get_body_name(body2))
            print("collision with environment")

    pb_ompl_interface.setup_collision_detection(robot, obstacles, self_collisions, allow_collision_links)

    valid = pb_ompl_interface.is_state_valid(robot.state)
    print("Start Stat is valid: ", valid)

    robot.set_robot(goal)
    valid = pb_ompl_interface.is_state_valid(robot.state)
    print("Goal Stat is valid: ", valid)

    robot.set_robot(start)

    # Planning
    res, path = pb_ompl_interface.plan(goal)
    if res:
        print("solution found")
        # pb_ompl_interface.execute(path)


