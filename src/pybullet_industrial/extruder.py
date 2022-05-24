import pybullet as p
import numpy as np
from pybullet_industrial.endeffector_tool import EndeffectorTool


class Extruder(EndeffectorTool):

    def __init__(self, urdf_model: str, start_position, start_orientation, extruder_properties, coupled_robots = None, tcp_frame=None, connector_frames=None):
        super().__init__(urdf_model, start_position, start_orientation, coupled_robots, tcp_frame, connector_frames)

        self.extruder_properties = {'opening angle':0,'number of rays':1,'maximum distance':1,'material':None}
        self.change_extruder_properties(extruder_properties)
        if self.extruder_properties['material'] is None:
            raise ValueError("The extruder requires a initial Material")

    def extrude(self,tcp_frame=None):
        position, orientation = self.get_tool_pose(tcp_frame)
        ray_cast_results = self.cast_rays(position, orientation,
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

    @staticmethod
    def cast_rays(position,orientation,opening_angle,number_of_rays,ray_length):
        ray_start_pos = []
        ray_end_pos = []

        phi = np.random.uniform(-np.pi,np.pi,number_of_rays)
        theta = np.random.uniform(-0.5*opening_angle,0.5*opening_angle,number_of_rays)
        x = np.sin(theta)* np.cos(phi)
        y = np.sin(theta)* np.sin(phi)
        z = np.cos(theta)
        ray_directions = np.array([x,y,z])

        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(number_of_rays):
            ray_start_pos.append(position)

            ray_dir = rot_matrix@ray_directions[:,i]

            ray_end_pos.append(position-ray_length*ray_dir)

        results = p.rayTestBatch(ray_start_pos, ray_end_pos)
        return results
