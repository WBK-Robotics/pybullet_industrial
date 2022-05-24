import os
import time
import pybullet as p
import pybullet_data
import pybullet_industrial as pi
import numpy as np
from lemniscate import build_lemniscate_path


class Material:
    def __init__(self):
        pass

    def spawn_particle(self,position):
        pass

class Plastic(Material):

    def __init__(self,particle_size,color):
        self.particle_size = particle_size
        self.color = color

        self.visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=self.color, radius=self.particle_size)
        self.collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=self.particle_size)

    def spawn_particle(self,position):
        particle = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=self.collisionShapeId,
                                     baseVisualShapeIndex=self.visualShapeId,
                                     basePosition=position)
        return particle


class Paint(Material):

    def __init__(self,particle_size,color):
        self.particle_size = particle_size
        self.color = color

        self.visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=self.color, radius=self.particle_size)

    def spawn_particle(self,position):
        particle = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=-1,
                                     baseVisualShapeIndex=self.visualShapeId,
                                     basePosition=position)
        return particle


class Extruder(pi.EndeffectorTool):

    def __init__(self, urdf_model: str, start_position, start_orientation, extruder_properties, coupled_robots = None, tcp_frame=None, connector_frames=None):
        super().__init__(urdf_model, start_position, start_orientation, coupled_robots, tcp_frame, connector_frames)

        self.extruder_properties = {'opening angle':0,'number of rays':1,'maximum distance':1,'material':None}
        self.change_extruder_properties(extruder_properties)
        if self.extruder_properties['material'] is None:
            raise ValueError("The extruder requires a initial Material")

    def extrude(self):
        ray_cast_results = cast_rays(position, orientation,
                                     self.extruder_properties['opening angle'],
                                     self.extruder_properties['number of rays'],
                                     self.extruder_properties['maximum distance'])
        for i in range(self.extruder_properties['number of rays']):
            ray_intersection = ray_cast_results[i][3]
            self.extruder_properties['material'].spawn_particle(ray_intersection)

    def change_extruder_properties(self,new_properties):
        for key in new_properties:
            if not key in self.extruder_properties:
                raise KeyError("The specified property keys are not valid"+
                               " Valid keys are: "+str(self.extruder_properties.keys()))
            else:
                self.extruder_properties[key]=new_properties[key]




def sample_spherical(npoints, opening_angle):
    phi = np.random.uniform(-np.pi,np.pi,npoints)
    theta = np.random.uniform(-0.5*opening_angle,0.5*opening_angle,npoints)
    x = np.sin(theta)* np.cos(phi)
    y = np.sin(theta)* np.sin(phi)
    z = np.cos(theta)
    vec = np.array([x,y,z])
    return vec


def cast_rays(position,orientation,opening_angle,numRays,rayLen):
    rayFrom = []
    rayTo = []

    ray_directions = sample_spherical(numRays,opening_angle)
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    for i in range(numRays):
        rayFrom.append(position)

        ray_dir = rot_matrix@ray_directions[:,i]

        rayTo.append(position-rayLen*ray_dir)

    results = p.rayTestBatch(rayFrom, rayTo)
    return results

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

    paint = Paint(0.03,[0, 0, 1, 1])
    plastic = Plastic(0.03,[1, 0, 0, 1])

    extruder_properties = {'maximum distance':0.5,'opening angle':np.pi/6,'material':paint,'number of rays':20}
    extruder = Extruder(
        urdf_file2, [1.9, 0, 1.2], start_orientation,extruder_properties)
    extruder.couple(robot, 'link6')

    target_position = np.array([1.9, 0, 0.3])
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    steps = 100
    test_path = build_lemniscate_path(target_position, steps, 0.3, 0.8)
    pi.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    p.setRealTimeSimulation(1)
    for i in range(20):
        extruder.set_tool_pose(test_path[:, 0], target_orientation)
        time.sleep(0.1)
    

    for i in range(steps):
            extruder.set_tool_pose(test_path[:, i], target_orientation)
            position, orientation = extruder.get_tool_pose()
            pi.draw_coordinate_system(position, orientation)
            extruder.extrude()

            time.sleep(0.005)

    extruder.change_extruder_properties({'material':plastic,'opening angle':0,'number of rays':1})
    while True:
        for i in range(steps):
            extruder.set_tool_pose(test_path[:, i], target_orientation)
            position, orientation = extruder.get_tool_pose()
            pi.draw_coordinate_system(position, orientation)
            extruder.extrude()

            time.sleep(0.005)


