import pybullet as p
import pybullet_industrial as pi
import os
import numpy as np
from typing import Dict


def calculate_hit_percentage(remover):
    position, orientation = remover.get_tool_pose()
    raycast_results = remover.cast_rays(position, orientation)
    hit_positions = sum(
        [raycast_result[0] != -1 for raycast_result in raycast_results])
    nonhit_positions = sum(
        [raycast_result[0] == -1 for raycast_result in raycast_results])
    return hit_positions/(hit_positions+nonhit_positions)


class MillingTool(pi.EndeffectorTool):

    def __init__(self, urdf_model: str, start_position: np.array, start_orientation: np.array,
                 raycast_properties: Dict, coupled_robot: pi.RobotBase = None,
                 tcp_frame: str = None, connector_frame: str = None):
        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robot, tcp_frame, connector_frame)

        self.properties = {'diameter': 0.1,
                           'rotation speed': 1,
                           'number of teeth': 5,
                           'height': 0.1,
                           'number of rays': 10}
        self.current_angle = 0
        if raycast_properties is not None:
            self.change_properties(raycast_properties)

    def mill(self, tcp_frame=None):
        if tcp_frame is None:
            tcp_frame = self.tcp_frame

        ray_start_pos = []
        ray_end_pos = []
        angle_between_teeth = 2*np.pi/self.properties['number of teeth']

        position, orientation = self.get_tool_pose(tcp_frame)
        rot_matrix = p.getMatrixFromQuaternion(orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        for i in range(self.properties['number of teeth']):
            end_point = self.properties['diameter']*np.array(
                [np.sin(i*angle_between_teeth), np.cos(i*angle_between_teeth), 0])
            for j in range(self.properties['number of rays']):
                height_adjustment = np.array(
                    [0, 0, j*self.properties['height']/self.properties['number of rays']])

                start_position = position+rot_matrix@height_adjustment
                end_position = position + \
                    rot_matrix@(end_point+height_adjustment)
                ray_start_pos.append(start_position)
                ray_end_pos.append(end_position)

        results = p.rayTestBatch(ray_start_pos, ray_end_pos)
        return results

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
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'maximum distance': 2.0,
                          'opening angle': 0.4,
                          'number of rays': 200}

    position = [0.00, 0.0, 0.6]
    corner_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    corner_remover.set_tool_pose(position, [0, 0, 0, 1])

    position = [0.00, 0.25, 0.6]
    edge_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    edge_remover.set_tool_pose(position, [0, 0, 0, 1])

    position = [0.25, 0.25, 0.6]
    full_remover = pi.Remover(
        urdf_file2, position, [0, 0, 0, 1], remover_properties)
    full_remover.set_tool_pose(position, [0, 0, 0, 1])

    p.setPhysicsEngineParameter(numSolverIterations=4,
                                minimumSolverIslandSize=1024)

    for _ in range(100):
        p.stepSimulation()

    p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
    # disable rendering during creation
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    size = 0.5
    pi.spawn_material_block([0, 0, 0],
                            [size, size, size],
                            pi.MetalVoxel,
                            {'particle size': size/4})

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    corner_average = 0
    edge_average = 0
    full_average = 0
    n = 1
    while (1):
        corner_average = corner_average + \
            (calculate_hit_percentage(corner_remover)-corner_average)/n
        edge_average = edge_average + \
            (calculate_hit_percentage(edge_remover)-edge_average)/n
        full_average = full_average + \
            (calculate_hit_percentage(full_remover)-full_average)/n
        print("Corner: {}".format(corner_average))
        print("Edge: {}".format(edge_average))
        print("Full: {}".format(full_average))

        p.stepSimulation()
        n += 1
