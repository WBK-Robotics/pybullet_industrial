import pybullet as p
import numpy as np


class Material:
    def __init__(self):
        pass

    def spawn_particle(self,ray_cast_result):
        pass

class Plastic(Material):

    def __init__(self,particle_size,color):
        self.particle_size = particle_size
        self.color = color

        self.visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=self.color, radius=self.particle_size)
        self.collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=self.particle_size)

    def spawn_particle(self,ray_cast_result):
        particle = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=self.collisionShapeId,
                                     baseVisualShapeIndex=self.visualShapeId,
                                     basePosition=ray_cast_result[3])
        return particle


class Paint(Material):

    def __init__(self,particle_size,color):
        self.particle_size = particle_size
        self.color = color

        self.visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=self.color, radius=self.particle_size)

    def spawn_particle(self,ray_cast_result):
        particle_ids = []
        target_id = ray_cast_result[0]
        if target_id != -1:
            target_link_id = ray_cast_result[1]
            if target_link_id == -1:
                target_position,target_orientation = p.getBasePositionAndOrientation(target_id)
            else:
                target_link_state = p.getLinkState(target_id,target_link_id)
                target_position = np.array(target_link_state[0])
                target_orientation = np.array(target_link_state[1])
            adj_target_position = np.array(ray_cast_result[3])-target_position
            
            steps = 3
            width = self.particle_size*500
            theta2              = np.linspace(-np.pi,  0, steps)
            phi2                = np.linspace( 0 ,  5 * 2*np.pi , steps)


            x_coord           = self.particle_size * np.sin(theta2) * np.cos(phi2) + adj_target_position[0]
            y_coord           = self.particle_size * np.sin(theta2) * np.sin(phi2) + adj_target_position[1]
            z_coord           = self.particle_size * np.cos(theta2)+ adj_target_position[2]

            path = np.array([x_coord,y_coord,z_coord])
            path_steps = len(path[0])
            for i in range(1, path_steps):
                current_point = path[:, i]
                previous_point = path[:, i-1]
                particle_ids.append(p.addUserDebugLine(current_point, previous_point,
                                lineColorRGB=self.color, lineWidth=width,lifeTime=0,parentObjectUniqueId=target_id,parentLinkIndex=target_link_id))
        return particle_ids