import pybullet as p
import pybullet_industrial as pi
import os
import numpy as np
from typing import Dict


class MillingTool(pi.EndeffectorTool):

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 raycast_properties: Dict, coupled_robot: pi.RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):
        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.properties = {'diameter': 0.05,
                           'rotation speed': 1,
                           'number of teeth': 5,
                           'height': 0.1,
                           'number of rays': 10}
        self.current_angle = 0
        if raycast_properties is not None:
            self.change_properties(raycast_properties)

    def get_cutting_state(self, ray_cast_result, tcp_frame=None):
        if tcp_frame is None:
            tcp_id = self._tcp_id
        else:
            tcp_id = self._convert_link_to_id(tcp_frame)

        link_state = p.getLinkState(self.urdf, tcp_id, computeLinkVelocity=1)
        cutting_speed = np.linalg.norm(link_state[7])

        ray_cast_per_teeth = [ray_cast_result[i:i+self.properties['number of rays']]
                              for i in range(0, len(ray_cast_result),
                                             self.properties['number of rays'])]

        # calculate the cutting depth based on the amount of rays that are hitting a body
        # if the cutting depth is zero, the tool is not cutting
        cutting_depth = [sum([ray_cast[0] != -1 for ray_cast in ray_cast_per_teeth[i]])
                         for i in range(self.properties['number of teeth'])]
        cutting_depth = np.array(
            cutting_depth)*self.properties['height']/self.properties['number of rays']

        return cutting_speed, cutting_depth

    def mill(self, tcp_frame=None):

        ray_start_pos = []
        ray_end_pos = []
        angle_between_teeth = 2*np.pi/self.properties['number of teeth']

        position, orientation = self.get_tool_pose(tcp_frame)
        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(self.properties['number of teeth']):
            end_point = self.properties['diameter']*np.array(
                [np.sin(i*angle_between_teeth+self.current_angle),
                 np.cos(i*angle_between_teeth+self.current_angle),
                 0])
            for j in range(self.properties['number of rays']):
                height_adjustment = np.array(
                    [0, 0, j*self.properties['height']/self.properties['number of rays']])

                start_position = position+rot_matrix@height_adjustment
                end_position = position + \
                    rot_matrix@(end_point+height_adjustment)

                p.addUserDebugLine(start_position, end_position,
                                   [1, 0, 0], 1, lifeTime=1)
                ray_start_pos.append(start_position)
                ray_end_pos.append(end_position)

        ray_cast_results = p.rayTestBatch(ray_start_pos, ray_end_pos)

        cutting_speed, cutting_depth = self.get_cutting_state(
            ray_cast_results, tcp_frame)
        cutting_force = self.force_model(cutting_speed,
                                         cutting_depth,
                                         self.properties['diameter'],
                                         self.properties['rotation speed'])
        self.apply_tcp_force(cutting_force, tcp_frame)

        self.current_angle = self.current_angle + \
            self.properties['rotation speed']

        removed_objects = []
        for ray_intersection in ray_cast_results:
            if ray_intersection[0] != -1:
                p.removeBody(ray_intersection[0])
                removed_objects.append(ray_intersection[0])
        return removed_objects

    @staticmethod
    def force_model(cutting_speed, cutting_depth, diameter, rotation_speed):
        force = np.zeros(3)
        return force

    def change_properties(self, new_properties: Dict):
        """Allows retroactive changes to the ray casting properties.

        Args:
            new_properties (Dict): A dictionary containing values and keys that
                                   should be changed
        Raises:
            KeyError: If a key is not a valid property
        """
        for key in new_properties:
            if not key in self.properties:
                raise KeyError("The specified property keys are not valid" +
                               " Valid keys are: "+str(self.properties.keys()))
            self.properties[key] = new_properties[key]


if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                             'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'diameter': 0.1,
                          'rotation speed': 2*np.pi/5,
                          'number of teeth': 5,
                          'height': 0.1,
                          'number of rays': 10}

    position = [0.00, 0.0, 0.6]
    milling_tool = MillingTool(
        urdf_file, position, [0, 0, 0, 1], remover_properties)
    milling_tool.set_tool_pose(position, [0, 0, 0, 1])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    while (1):
        pi.Plastic([0, 0, 0, position+np.array([0., 0.05, 0])],
                   {'particle size': 0.02, 'color': [1, 0, 0, 1]})
        results = milling_tool.mill()
        p.stepSimulation()
