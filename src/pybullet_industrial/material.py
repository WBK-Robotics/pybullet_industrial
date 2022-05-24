import pybullet as p



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
        particle = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=-1,
                                     baseVisualShapeIndex=self.visualShapeId,
                                     basePosition=ray_cast_result[3])
        return particle