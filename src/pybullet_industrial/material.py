from typing import Dict

import numpy as np
import pybullet as p


class Particle():
    """A template class for material particles extruded by a extruder endeffector tool.

    All materials should inherit from this class and implement its methods.

    Args:
        ray_cast_result (list): The result of a pybullet ray_cast
                                as performed by the Extruder class.
                                It is made up of: [objectUniqueId,
                                                    linkIndex,
                                                    hit fraction,
                                                    hit position,
                                                    hit normal]
        material_properties (Dict): A dictionary containing the properties of the material
    """

    def __init__(self, ray_cast_result: list, material_properties: Dict):

        self.properties = {}

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
            self.properties[key] = new_properties[key]


class Plastic(Particle):
    """A class for simple particles which can be used for additive manufacturing.
        The particles are infinitely rigid and stick to each other.

    Args:
        ray_cast_result (list): The result of a pybullet ray_cast
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

    def __init__(self, ray_cast_result: list,  material_properties: Dict):

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


class Paint(Particle):
    """A class for simple Paint particles which stick to objects and move with them.
        The Paint particles are purely visible and have neither mass nor a collision mesh

    Args:
        ray_cast_result (list): The result of a pybullet ray_cast as performed by the extruder
        material_properties (Dict): A dictionary containing the properties of the material.
                                    The default properties for Paint are:
                                    'particle size': 0.3, 'color': [1, 0, 0, 1]
    """

    def __init__(self, ray_cast_result: list, material_properties: Dict):

        self.properties = {'particle size': 0.3, 'color': [1, 0, 0, 1]}
        self.set_material_properties(material_properties)
        particle_size = self.properties['particle size']
        color = self.properties['color']

        self.particle_ids = []
        self.target_id = ray_cast_result[0]
        self.target_link_id = ray_cast_result[1]
        self.hit_position = ray_cast_result[3]
        self.initial_target_position = self.get_target_pose(
            self.target_id, self.target_link_id)[0]

        if self.target_id != -1:
            target_position, target_orientation = self.get_target_pose(
                self.target_id, self.target_link_id)

            rot_matrix = p.getMatrixFromQuaternion(target_orientation)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)

            self.adj_target_position = np.linalg.inv(
                rot_matrix)@(np.array(ray_cast_result[3])-target_position)

            steps = 3
            width = particle_size*500
            theta2 = np.linspace(-np.pi,  0, steps)
            phi2 = np.linspace(0,  5 * 2*np.pi, steps)

            x_coord = particle_size * \
                np.sin(theta2) * np.cos(phi2) + self.adj_target_position[0]
            y_coord = particle_size * \
                np.sin(theta2) * np.sin(phi2) + self.adj_target_position[1]
            z_coord = particle_size * \
                np.cos(theta2) + self.adj_target_position[2]

            path = np.array([x_coord, y_coord, z_coord])
            path_steps = len(path[0])
            for i in range(1, path_steps):
                current_point = path[:, i]
                previous_point = path[:, i-1]
                self.particle_ids.append(p.addUserDebugLine(current_point, previous_point,
                                                            lineColorRGB=color[:3],
                                                            lineWidth=width,
                                                            lifeTime=0,
                                                            parentObjectUniqueId=self.target_id,
                                                            parentLinkIndex=self.target_link_id))

    @staticmethod
    def get_target_pose(target_id: int, target_link_id: int):
        """Returns the pose of a target objects link.

        This function is used to calculate the relative position
        of the paint particle to the object it sticks to.

        Args:
            target_id (int): The unique id of the target object
            target_link_id (int): The link id of the target object

        Returns:
            np.array, np.array: The position and orientation of the target object
        """
        if target_link_id == -1:
            target_position, target_orientation = p.getBasePositionAndOrientation(
                target_id)
        else:
            target_link_state = p.getLinkState(target_id, target_link_id)
            target_position = np.array(target_link_state[0])
            target_orientation = np.array(target_link_state[1])
        return target_position, target_orientation

    def get_position(self):
        """Returns the position of a particle in the world frame

        Returns:
            [float,float,float]: The three dimensional position of the particle
                                 in the world coordinate system
        """
        hit_position = self.hit_position
        initial_target_position = self.initial_target_position
        diff_vector = np.array(hit_position)-np.array(initial_target_position)

        target_position, target_orientation = self.get_target_pose(
            self.target_id, self.target_link_id)

        rot_matrix = p.getMatrixFromQuaternion(target_orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        coordinates = target_position+rot_matrix @ (diff_vector)

        return coordinates

    def remove(self):
        """Function to actively remove the particle from the simulation.
           This function is deliberatly kept seperate from the __del__ method to prevent having
           to manually save particles if there is no intention of removing them.
        """
        [p.removeUserDebugItem(id) for id in self.particle_ids]


class MetalVoxel(Particle):
    """A simple voxel class for cutting and milling simulations

    Args:
        ray_cast_result (list): The result of a pybullet ray_cast
                                as performed by the Extruder class.
        material_properties (Dict): A dictionary containing the properties of the material.
                                    The default properties for a Metal Voxel are:
                                    'particle size': 0.3, 'color': [1, 0, 0, 1]
    """

    def __init__(self, ray_cast_result: list,  material_properties: Dict):

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

        Returns:
            [float,float,float]: The three dimensional position of the particle
                                 in the world coordinate system
        """
        position, _ = p.getBasePositionAndOrientation(self.particle_id)
        return position

    def remove(self):
        p.removeBody(self.particle_id)


def spawn_material_block(base_position: list, dimensions: list,
                         material: Particle, material_properties: Dict):
    """Spawns a block of a give material.

    Args:
        base_position ([float,float,float]): The position of the lower left base corner of the block
        dimensions ([float,float,float]): The dimensions of the block in [width,breath,height]
        material (Particle): A particle that should be spawned
        material_properties (Dict): A dictionary containing the properties of the material.
                                    It needs to contain a key 'particle size'.

    Returns:
        list[Particle]: A list of the spawned particles
    """
    if 'particle size' not in material_properties.keys():
        raise KeyError(
            "The material properties must contain the key 'particle size'!")
    particle_size = material_properties['particle size']
    half_extents = particle_size*0.5

    batch_positions = []

    for x in range(int(dimensions[0]/particle_size)):
        for y in range(int(dimensions[1]/particle_size)):
            for z in range(int(dimensions[2]/particle_size)):
                batch_positions.append(
                    [x * particle_size+base_position[0]+half_extents,
                     y * particle_size+base_position[1]+half_extents,
                     z * particle_size+base_position[2]+half_extents])

    objects = []
    for positions in batch_positions:
        particle = material([0, 0, 0, positions], material_properties)
        objects.append(particle)

    return objects
