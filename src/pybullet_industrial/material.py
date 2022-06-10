import pybullet as p
import numpy as np


class Particle():
    def __init__(self, ray_cast_result, particle_size, color):
        pass

    def get_position(self):
        pass

    def remove(self):
        pass


class Plastic(Particle):

    def __init__(self, ray_cast_result, particle_size, color):

        visualShapeId = p.createVisualShape(
            shapeType=p.GEOM_SPHERE, rgbaColor=color, radius=particle_size)
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE, radius=particle_size)

        self.particle_id = p.createMultiBody(baseMass=0,
                                             baseCollisionShapeIndex=collisionShapeId,
                                             baseVisualShapeIndex=visualShapeId,
                                             basePosition=ray_cast_result[3])

    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.particle_id)
        return position

    def remove(self):
        p.removeBody(self.particle_id)


class Paint(Particle):

    def __init__(self, ray_cast_result, particle_size, color):

        self.particle_ids = []
        target_id = ray_cast_result[0]
        if target_id != -1:
            target_link_id = ray_cast_result[1]
            if target_link_id == -1:
                target_position, target_orientation = p.getBasePositionAndOrientation(
                    target_id)
            else:
                target_link_state = p.getLinkState(target_id, target_link_id)
                target_position = np.array(target_link_state[0])
                target_orientation = np.array(target_link_state[1])

            rot_matrix = p.getMatrixFromQuaternion(target_orientation)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)

            adj_target_position = np.linalg.inv(
                rot_matrix)@(np.array(ray_cast_result[3])-target_position)

            steps = 3
            width = particle_size*500
            theta2 = np.linspace(-np.pi,  0, steps)
            phi2 = np.linspace(0,  5 * 2*np.pi, steps)

            x_coord = particle_size * \
                np.sin(theta2) * np.cos(phi2) + adj_target_position[0]
            y_coord = particle_size * \
                np.sin(theta2) * np.sin(phi2) + adj_target_position[1]
            z_coord = particle_size * \
                np.cos(theta2) + adj_target_position[2]

            path = np.array([x_coord, y_coord, z_coord])
            path_steps = len(path[0])
            for i in range(1, path_steps):
                current_point = path[:, i]
                previous_point = path[:, i-1]
                self.particle_ids.append(p.addUserDebugLine(current_point, previous_point,
                                                            lineColorRGB=color[:3],
                                                            lineWidth=width,
                                                            lifeTime=0,
                                                            parentObjectUniqueId=target_id,
                                                            parentLinkIndex=target_link_id))

    def get_position(self):
        return 1

    def remove(self):
        [p.removeUserDebugItem(id) for id in self.particle_ids]
