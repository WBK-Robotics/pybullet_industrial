import os

import numpy as np
import pybullet as p
import pybullet_industrial as pi

if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file = os.path.join(dirname,
                             'robot_descriptions', 'milling_head.urdf')
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_nj290_robot.urdf')
    cid = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    remover_properties = {'diameter': 0.1,
                          'rotation speed': 2*np.pi/5,
                          'number of teeth': 5,
                          'height': 0.1,
                          'number of rays': 1}
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0, 0, 0, 1])
    position = [0.00, 0.0, 0.6]
    milling_tool = pi.MillingTool(
        urdf_file, position, [0, 0, 0, 1], remover_properties)
    milling_tool.set_tool_pose(position, [0, 0, 0, 1])
    milling_tool.couple(robot, 'link6')

    test_path = pi.linear_interpolation(
        [2.3, -1.8, 1.2], [2.3, 1.8, 1.2], 100)

    for _ in range(500):
        milling_tool.set_tool_pose(*test_path.get_start_pose())
        p.stepSimulation()

    speeds = []
    for steps in test_path:
        milling_tool.set_tool_pose(steps[0], [0, 0, 0, 1])
        print(steps[0])
        p.stepSimulation()
        tool_position, tool_orientation = milling_tool.get_tool_pose()
        ray_cast_results = milling_tool.cast_rays(
            tool_position, tool_orientation)
        cutting_speed, _ = milling_tool.get_cutting_state(
            ray_cast_results)
        speeds.append(cutting_speed)

    p.disconnect()

    import matplotlib.pyplot as plt

    plt.plot(speeds)
    plt.show()