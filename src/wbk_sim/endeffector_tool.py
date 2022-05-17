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
        if tcp_frame == None:
            last_link = max(self._link_name_to_index)
            self._tcp_id = self._link_name_to_index[last_link]
        else:
            self._tcp_id = self._convert_endeffector(tcp_frame)

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

    def get_tool_pose(self):
        # converts between the robots forward kinematics and the tool forward kinematics
        pass

    def set_tool_pose(self, target_position, target_orientation=None, tcp_frame=None):
        if tcp_frame is None:
            tcp_frame = self._tcp_id
        else:
            tcp_frame = self._link_name_to_index[tcp_frame]

        translation, rotation = self.compute_relative_transformation(tcp_frame)

        adj_target_position = target_position+translation

        self._coupled_robot.set_endeffector_pose(
            adj_target_position, endeffector_name=self._coupling_link)

    def compute_relative_transformation(self, tcp_frame):
        base_link_state = p.getLinkState(
            self.urdf, 0, computeForwardKinematics=True)
        base_pos = np.array(base_link_state[0])
        base_ori = np.array(base_link_state[1])
        base_ori_matrix = p.getMatrixFromQuaternion(base_ori)
        base_ori_matrix = np.array(base_ori_matrix).reshape(3, 3)

        tcp_link_state = p.getLinkState(
            self.urdf, tcp_frame, computeForwardKinematics=True)
        tcp_pos = np.array(tcp_link_state[0])
        tcp_ori = np.array(tcp_link_state[1])
        tcp_ori_matrix = p.getMatrixFromQuaternion(tcp_ori)
        tcp_ori_matrix = np.array(tcp_ori_matrix).reshape(3, 3)

        translation = tcp_pos-base_pos
        orientation = tcp_ori_matrix@np.linalg.inv(base_ori_matrix)

        return translation, orientation


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
        urdf_file2, [0, 0, 0], start_orientation, tcp_frame='tcp')
    milling_head.couple(robot, 'link6')
    p.setRealTimeSimulation(1)

    target_position = [1.9, 0, 1.2]
    test_path = build_lemniscate_path(target_position, 400, 1.2, 1)
    wbk.draw_path(test_path)
    target_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    while True:
        for i in range(400):
            # robot.set_endeffector_pose(
            #    test_path[:, i], target_orientation, 'link6')
            milling_head.set_tool_pose(test_path[:, i])
            wbk.draw_coordinate_system(test_path[:, i], target_orientation)
            time.sleep(0.005)
