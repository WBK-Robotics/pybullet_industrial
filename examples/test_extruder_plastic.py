import os
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np
from copy import deepcopy



def build_vertical_stack(start_position,extruder,step_num,step_size):
    target_position = deepcopy(start_position)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    for i in range(20):
        extruder.set_tool_pose(target_position, target_orientation)
        for _ in range(50):
                p.stepSimulation()
    
    relative_path = np.ones((3,step_num))*np.inf
    for i in range(step_num):
        target_position[2] = target_position[2]+step_size
        extruder.set_tool_pose(target_position, target_orientation)
        for _ in range(10):
            p.stepSimulation()
        position, _ = extruder.get_tool_pose()
        id=extruder.extrude()
        if id:
            material_position,_ = p.getBasePositionAndOrientation(id[0])
            relative_path[:,i] = position-material_position
        
    return relative_path




if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    monastryId =p.createCollisionShape(p.GEOM_MESH,
                            fileName="samurai_monastry.obj",
                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    plasstic_diameter = 0.03
    plastic = pi.Plastic(plasstic_diameter,[1, 0, 0, 1])

    extruder_properties = {'maximum distance':0.5,'opening angle':0,'material':plastic,'number of rays':1}
    extruder = pi.Extruder(
        urdf_file2, [1.9, 0, 1.2], start_orientation,extruder_properties)
    extruder.couple(robot, 'link6')


    #target_position = np.array([1.9, -0.3,plasstic_diameter/2])
    #print(build_vertical_stack(target_position,extruder,20,plasstic_diameter/2))
    #target_position = np.array([1.9, 0,0.2])# should be zero
    #build_vertical_stack(target_position,extruder,20,plasstic_diameter/2) # should steadily decrease with constant factor
    target_position = np.array([1.9, 0.3,0.6]) 
    print(build_vertical_stack(target_position,extruder,20,plasstic_diameter/2)) #  should be inf
    extruder.change_extruder_properties({'maximum distance':0.7})
    print(build_vertical_stack(target_position,extruder,20,plasstic_diameter/2)) #  should not be inf



