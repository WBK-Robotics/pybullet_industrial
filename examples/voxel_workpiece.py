import pybullet as p
import pybullet_industrial as pi
from typing import Dict
import numpy as np
import os


class Remover(pi.EndeffectorTool):

    def __init__(self, urdf_model: str, start_position, start_orientation, remover_properties: Dict, coupled_robots=None, tcp_frame=None, connector_frames=None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robots, tcp_frame, connector_frames)

        self.remover_properties = {'opening angle': 0,
                                   'number of rays': 1,
                                   'maximum distance': 1}

        self.change_remover_properties(remover_properties)

    def remove(self, tcp_frame=None):
        """Extrudes material from the specified tcp_frame.

        Args:
            tcp_frame (str, optional): the name of the link from which to extrude the material.
                                       Defaults to None in which case the default tcp is used
        """
        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation,
                                          self.remover_properties['opening angle'],
                                          self.remover_properties['number of rays'],
                                          self.remover_properties['maximum distance'])

        removed_objects = []
        for i in range(self.remover_properties['number of rays']):
            ray_intersection = ray_cast_results[i]
            if ray_intersection[0] != -1:
                p.removeBody(ray_intersection[0])

                removed_objects.append(ray_intersection[0])
        return removed_objects

    def change_remover_properties(self, new_properties: Dict):
        """Allows retroactive changes to the extruder properties.


        Args:
            new_properties (Dict): A dictionary containing values and keys that
                                   should be changed

        Raises:
            KeyError: If a key is not a valid extruder property
        """
        for key in new_properties:
            if not key in self.remover_properties:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.remover_properties.keys()))
            else:
                self.remover_properties[key] = new_properties[key]

    @staticmethod
    def cast_rays(position, orientation, opening_angle, number_of_rays, ray_length):
        """Randomly casts rays withn a given range from a specified position and orientation

        Args:
            position ([type]): start position of the rays
            orientation ([type]): start orientation of the rays
            opening_angle ([type]): the opening angle in which the rays should be cast
            number_of_rays ([type]): The number of rays which are cast
            ray_length ([type]): The maximum range of the rays

        Returns:
            [type]: [description]
        """
        ray_start_pos = []
        ray_end_pos = []

        phi = np.random.uniform(-np.pi, np.pi, number_of_rays)
        theta = np.random.uniform(-0.5*opening_angle,
                                  0.5*opening_angle, number_of_rays)
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)
        ray_directions = np.array([x, y, z])

        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(number_of_rays):
            ray_start_pos.append(position)

            ray_dir = rot_matrix@ray_directions[:, i]

            ray_end_pos.append(position-ray_length*ray_dir)

        results = p.rayTestBatch(ray_start_pos, ray_end_pos)
        return results


def spawn_voxel_block(base_position, dimensions, voxel_size, color=[1, 1, 1, 1]):
    half_extents = voxel_size*0.5
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        rgbaColor=color,
                                        halfExtents=[half_extents, half_extents, half_extents])
    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                              halfExtents=[half_extents, half_extents, half_extents])

    batchPositions = []

    for x in range(int(dimensions[0]/voxel_size)):
        for y in range(int(dimensions[1]/voxel_size)):
            for z in range(int(dimensions[2]/voxel_size)):
                batchPositions.append(
                    [x * voxel_size+base_position[0]+half_extents,
                     y * voxel_size+base_position[1]+half_extents,
                     z * voxel_size+base_position[2]+half_extents])

    bodyUids = p.createMultiBody(baseMass=0.0,
                                 baseInertialFramePosition=[0, 0, 0],
                                 baseCollisionShapeIndex=collisionShapeId,
                                 baseVisualShapeIndex=visualShapeId,
                                 batchPositions=batchPositions,
                                 useMaximalCoordinates=True)

    return bodyUids


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'maximum distance': 2.0,
                          'opening angle': 0.0,
                          'number of rays': 1}

    position = [0.01, -0.5, 1.2]
    remover = Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)

    p.setPhysicsEngineParameter(numSolverIterations=4,
                                minimumSolverIslandSize=1024)

    p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
    # disable rendering during creation
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    size_progression = [1, 0.5, 0.2, 0.1, 0.05, 0.02]
    start = 0
    for size in size_progression:
        spawn_voxel_block([0, start, 0],
                          [size, size, size],
                          size/10)
        start += size

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while (1):
        remover.remove()
        remover.remove()
        remover.remove()
        remover.remove()
        position[1] += 0.005
        remover.set_tool_pose(position, [0, 0, 0, 1])
        p.stepSimulation()
        objectUid, object_index = pi.get_object_id_from_mouse()
        if (objectUid >= 0):
            p.removeBody(objectUid)
