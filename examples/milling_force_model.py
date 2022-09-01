import os

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                             'robot_descriptions', 'milling_head.urdf')

    cid = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'diameter': 0.1,
                          'rotation speed': 2*np.pi/5,
                          'number of teeth': 6,
                          'height': 0.1,
                          'number of rays': 10}

    position = [0.00, 0.0, 0.0]
    milling_tool = pi.MillingTool(
        urdf_file, position, [0, 0, 0, 1], remover_properties)
    milling_tool.set_tool_pose(position, [0, 0, 0, 1])

    pi.MetalVoxel([0, 0, 0, position+np.array([0.0, 0.06, 0])],
                  {'particle size': 0.1, 'color': [1, 0, 0, 1]})
    # pi.Plastic([0, 0, 0, position+np.array([0.05, 0., 0])],
    #           {'particle size': 0.03, 'color': [1, 0, 0, 1]})

    for _ in range(100):
        p.stepSimulation()
    tool_position, tool_orientation = milling_tool.get_tool_pose()

    force_x = []
    for i in range(10):
        ray_cast_results = milling_tool.cast_rays(
            tool_position, tool_orientation)
        cutting_speed, cutting_depth = milling_tool.get_cutting_state(
            ray_cast_results)

        teeth_angles = [i * 2*np.pi/milling_tool.properties['number of teeth'] +
                        milling_tool.current_angle for i in range(milling_tool.properties['number of teeth'])]

        cutting_force = milling_tool.force_model(0.01,
                                                 cutting_depth,
                                                 milling_tool.properties['diameter'],
                                                 milling_tool.properties['rotation speed'],
                                                 milling_tool.properties['material_specific_force'],
                                                 milling_tool.properties['chip_thickness_exponent'],
                                                 teeth_angles)
        # print(cutting_force)
        ray_cast_results = milling_tool.cast_rays(
            tool_position, tool_orientation)

        milling_tool.current_angle += np.pi*2/5*0.1

        print(cutting_force[0], cutting_force[1])
        force_x.append(cutting_force[0])
        p.stepSimulation()

    import matplotlib.pyplot as plt

    plt.plot(force_x)
    print(np.mean(force_x))
    plt.show()
