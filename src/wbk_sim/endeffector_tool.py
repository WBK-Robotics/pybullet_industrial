import pybullet as p
import numpy as np
from wbk_sim import RobotBase


class EndeffectorTool:
    def __init__(self, urdf_model: str, start_position, start_orientation, coupled_robot: RobotBase = None, tcp_frame=None):
        """The base class for all Tools and Sensors connected to a Robot

        Args:
            urdf_model (str): A valid path to a urdf file describint the tool geometry
            start_position ([type]): the position at which the tool should be spawned
            start_orientation ([type]): the orientation at which the tool should be spawned
            coupled_robot ([type], optional): A wbk_sim.Robot object if the robot is currently coupled. 
                                              Defaults to None.
            tcp_frame ([type], optional): The name of the urdf_link describing the tool center point.
                                          Defaults to None.
        """
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._coupled_robot = None
        self._coupling_link = None
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                       -1, -1, -1,
                                                       p.JOINT_FIXED,
                                                       [0, 0, 0],
                                                       [0, 0, 0],
                                                       start_position,
                                                       start_orientation)
        if not coupled_robot is None:
            self.couple(coupled_robot)

        self._link_name_to_index = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf, joint_number)[
                12].decode("utf-8")
            self._link_name_to_index[link_name] = joint_number

        if tcp_frame is None:
            last_link = max(self._link_name_to_index)
            self._tcp_id = self._link_name_to_index[last_link]
        else:
            self._tcp_id = self._convert_tcp(tcp_frame)

    def couple(self, robot, endeffector_name=None):
        """Dynamically Couples the Tool with the Endeffector of a given robot.
        Note that this endeffector can also be a virtual link to connect a sensor.
        A Tool can only be coupled with one robot

        Args:
            robot (wbk_sim.robot): The robot whith which the tool should couple.
            endeffector_name (str, optional): The endeffector of the robot where the tool should couple to. 
                                              Defaults to None.

        Raises:
            Error: [description]
        """
        # creates a fixed constrained between the choosen robot and the tool
        if self._coupled_robot is not None:
            raise ValueError("The Tool is already coupled with a robot")
        else:
            if endeffector_name == None:
                endeffector_index = robot._default_endeffector_id
            else:
                endeffector_index = robot._convert_endeffector(
                    endeffector_name)
            self._coupled_robot = robot
            self._coupling_link = endeffector_name
            p.removeConstraint(self._coupling_constraint)
            self._coupling_constraint = p.createConstraint(self._coupled_robot.urdf, endeffector_index,
                                                           self.urdf, -1,
                                                           p.JOINT_FIXED,
                                                           [0, 0, 0],
                                                           [0, 0, 0],
                                                           [0, 0, 0],
                                                           [0, 0, 0])

    def is_coupled(self):
        """Function which returns true if the Tool is currently coupled to a robot

        Returns:
            bool: 1 if the tool is coupled, 0 if not
        """
        if self._coupled_robot is None:
            return 0
        else:
            return 1

    def decouple(self):
        """Decouples the tool from the current robot. 
           In this case a new constraint is created rooting the tool in its current pose. 
        """
        self._coupled_robot = None
        self._coupling_link = None
        p.removeConstraint(self._coupling_constraint)
        position, orientation = p.getBasePositionAndOrientation(self.urdf)
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                       -1, -1, -1,
                                                       p.JOINT_FIXED,
                                                       [0, 0, 0],
                                                       [0, 0, 0],
                                                       position,
                                                       orientation)
        pass

    def get_tool_pose(self, tcp_frame: str = None):
        if tcp_frame is None:
            tcp_id = self._tcp_id
        else:
            tcp_id = self._convert_tcp(tcp_frame)

        link_state = p.getLinkState(self.urdf, tcp_id)

        position = np.array(link_state[0])
        orientation = np.array(link_state[1])
        return position, orientation

    def set_tool_pose(self, target_position, target_orientation=None, tcp_frame=None):

        translation, rotation = self.compute_relative_transformation(tcp_frame)

        adj_target_position = target_position-translation

        if not target_orientation is None:
            adj_target_orientation = quaternion_multiply(
                target_orientation, quaternion_inverse(rotation))
        else:
            adj_target_orientation = None

        print(rotation)
        print(target_orientation, adj_target_orientation)
        self._coupled_robot.set_endeffector_pose(
            adj_target_position, adj_target_orientation, endeffector_name=self._coupling_link)

    def compute_relative_transformation(self, tcp_frame):
        base_pos, base_ori = p.getBasePositionAndOrientation(self.urdf)

        tcp_pos, tcp_ori = self.get_tool_pose(tcp_frame)

        translation = tcp_pos-base_pos
        rotation = quaternion_multiply(
            quaternion_inverse(tcp_ori), base_ori)
        return translation, rotation

    def _convert_tcp(self, tcp):
        if isinstance(tcp, str):
            if tcp in self._link_name_to_index.keys():
                return self._link_name_to_index[tcp]
            else:
                ValueError("Invalid TCP name! valid names are: " +
                           str(self._link_name_to_index.keys()))
        else:
            raise TypeError(
                "The TCP must be a String describing a URDF link")


def quaternion_inverse(quaternion):
    q = np.array(quaternion, copy=True)
    np.negative(q[1:], q[1:])
    return q / np.dot(q, q)


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0, x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0, x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
    ])


if __name__ == "__main__":
    import os
    import time
    import pybullet as p
    import wbk_sim as wbk
    from lemniscate import build_lemniscate_path
    import numpy as np

    dirname = os.path.dirname(__file__)
    parentDir = os.path.dirname(dirname)
    parentDir = os.path.dirname(parentDir)
    urdf_file1 = os.path.join(parentDir, 'examples',
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(parentDir, 'examples',
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = wbk.RobotBase(urdf_file1, [0, 0, 0], start_orientation)
    milling_head = EndeffectorTool(
        urdf_file2, [1.9, 0, 1.2], start_orientation)
    milling_head.couple(robot, 'link6')

    target_position = [1.9, 0, 1.2]
    test_path = build_lemniscate_path(target_position, 400, 1.2, 0.5)
    wbk.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    p.setRealTimeSimulation(1)
    while True:
        for i in range(400):
            milling_head.set_tool_pose(test_path[:, i], None)
            wbk.draw_coordinate_system(test_path[:, i], target_orientation)
            time.sleep(0.005)
