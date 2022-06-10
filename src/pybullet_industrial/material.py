import pybullet as p
import numpy as np
from typing import Dict


class Particle():
    def __init__(self, ray_cast_result, material_properties: Dict):
        """A template class for material particles extruded by a extruder endeffector tool

        Args:
            ray_cast_result ([type]): The result of a pybullet ray_cast as performed by the extruder
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
            ray_cast_result ([type]): The result of a pybullet ray_cast as performed by the extruder
            material_properties (Dict): A dictionary containing the properties of the material.
                                        The default properties for Plastic are:
                                        'particle size': 0.3, 'color': [1, 0, 0, 1]
        """
        self.properties = {'particle size': 0.3, 'color': [1, 0, 0, 1]}
        self.set_material_properties(material_properties)
        particle_size = self.properties['particle size']
        color = self.properties['color']

        visualShapeId = p.createVisualShape(
            shapeType=p.GEOM_SPHERE, rgbaColor=color, radius=particle_size)
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE, radius=particle_size)

        self.particle_id = p.createMultiBody(baseMass=0,
                                             baseCollisionShapeIndex=collisionShapeId,
                                             baseVisualShapeIndex=visualShapeId,
                                             basePosition=ray_cast_result[3])

    def get_position(self):
        """Returns the position of a particle in the world frame
        """
        position, _ = p.getBasePositionAndOrientation(self.particle_id)
        return position

    def remove(self):
        p.removeBody(self.particle_id)
