from typing import Dict

import pybullet as p


class Particle():
    def __init__(self, ray_cast_result, material_properties: Dict):
        """A template class for material particles extruded by a extruder endeffector tool

        Args:
            ray_cast_result ([type]): The result of a pybullet ray_cast 
                                      as performed by the Extruder class. 
                                      It is made up of: [objectUniqueId, 
                                                         linkIndex,
                                                         hit fraction,
                                                         hit position,
                                                         hit normal]
            material_properties (Dict): A dictionary containing the properties of the material
        """
        self.properties = {}
        pass

    def get_position(self):
        """Returns the position of a particle in the world frame
        """
        pass

    def remove(self):
        """Function to actively remove the particle from the simulation.
           This function is deliberatly kept seperate from the __del__ method to prevent having
           to manually save particles if there is no intention of removing them.
        """
        pass

    def set_material_properties(self, new_properties: Dict):
        """Checks if the matieral properties contain the proper keys


        Args:
            new_properties (Dict): A dictionary containing the material properties

        Raises:
            KeyError: If a key is not a valid extruder property
        """
        for key in new_properties:
            if not key in self.properties:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.properties.keys()))
            else:
                self.properties[key] = new_properties[key]


class Plastic(Particle):

    def __init__(self, ray_cast_result,  material_properties: Dict):
        """A class for simply Plastic particles which can be used for 3d Printing.
           The particles are infinitely rigid and stick to each other.

        Args:
            ray_cast_result ([type]): The result of a pybullet ray_cast
                                      as performed by the Extruder class.
                                      It is made up of: [objectUniqueId, 
                                                         linkIndex,
                                                         hit fraction,
                                                         hit position,
                                                         hit normal]
            material_properties (Dict): A dictionary containing the properties of the material.
                                        The default properties for Plastic are:
                                        'particle size': 0.3, 'color': [1, 0, 0, 1]
        """
        self.properties = {'particle size': 0.3, 'color': [1, 0, 0, 1]}
        self.set_material_properties(material_properties)
        particle_size = self.properties['particle size']
        color = self.properties['color']

        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE, rgbaColor=color, radius=particle_size)
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE, radius=particle_size)

        self.particle_id = p.createMultiBody(baseMass=0,
                                             baseCollisionShapeIndex=collision_shape_id,
                                             baseVisualShapeIndex=visual_shape_id,
                                             basePosition=ray_cast_result[3])

    def get_position(self):
        """Returns the position of a particle in the world frame

        Returns:
            [float,float,float]: The three dimensional position of the particle 
                                 in the world coordinate system 
        """
        position, _ = p.getBasePositionAndOrientation(self.particle_id)
        return position

    def remove(self):
        p.removeBody(self.particle_id)


class MetalVoxel(Particle):
    def __init__(self, ray_cast_result,  material_properties: Dict):
        """A simple voxel class for cutting and milling simulations

        Args:
            ray_cast_result ([type]): The result of a pybullet ray_cast
                                      as performed by the Extruder class.
                                      It is made up of: [objectUniqueId, 
                                                         linkIndex,
                                                         hit fraction,
                                                         hit position,
                                                         hit normal]
            material_properties (Dict): A dictionary containing the properties of the material.
                                        The default properties for Plastic are:
                                        'particle size': 0.3, 'color': [1, 0, 0, 1]
        """
        self.properties = {'particle size': 0.3, 'color': [1, 0, 0, 1]}
        self.set_material_properties(material_properties)
        particle_size = self.properties['particle size']
        color = self.properties['color']

        half_extents = particle_size*0.5
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                              rgbaColor=color,
                                              halfExtents=[half_extents,
                                                           half_extents,
                                                           half_extents])
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[half_extents,
                                                                 half_extents,
                                                                 half_extents])

        self.particle_id = p.createMultiBody(baseMass=0,
                                             baseCollisionShapeIndex=collision_shape_id,
                                             baseVisualShapeIndex=visual_shape_id,
                                             basePosition=ray_cast_result[3])

    def get_position(self):
        """Returns the position of a particle in the world frame
        """
        position, _ = p.getBasePositionAndOrientation(self.particle_id)
        return position

    def remove(self):
        p.removeBody(self.particle_id)


def spawn_material_block(base_position, dimensions, material, material_properties):
    """Spawns a block of a give material.

    Args:
        base_position ([float,float,float]): The position of the lower left base corner of the block
        dimensions ([float,float,float]): The dimensions of the block in [width,breath,height]
        material (Particle): A particle that should be spawned 
        material_properties (Dict): A dictionary containing the properties of the material.
                                    It needs to contain a key 'particle size'.

    Returns:
        _type_: _description_
    """
    if 'particle size' not in material_properties.keys():
        raise KeyError(
            "The material properties must contain the key 'particle size'!")
    particle_size = material_properties['particle size']
    half_extents = particle_size*0.5

    batchPositions = []

    for x in range(int(dimensions[0]/particle_size)):
        for y in range(int(dimensions[1]/particle_size)):
            for z in range(int(dimensions[2]/particle_size)):
                batchPositions.append(
                    [x * particle_size+base_position[0]+half_extents,
                     y * particle_size+base_position[1]+half_extents,
                     z * particle_size+base_position[2]+half_extents])

    objects = []
    for positions in batchPositions:
        particle = material([0, 0, 0, positions], material_properties)
        objects.append(particle)

    return particle
