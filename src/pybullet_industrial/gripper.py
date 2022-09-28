import numpy as np
import pybullet as p

from pybullet_industrial.endeffector_tool import EndeffectorTool


class Gripper(EndeffectorTool):
    """The base class for all Tools and Sensors connected to a Robot

    Args:
        urdf_model (str): A valid path to a urdf file describing the tool geometry
        start_position (np.array): the position at which the tool should be spawned
        start_orientation (np.array): the orientation at which the tool should be spawned
        coupled_robots (pi.RobotBase, optional): A wbk_sim.Robot object if
                                                 the robot is coupled from the start.
                                                 Defaults to None.
        tcp_frame (str, optional): The name of the urdf_link
                                        describing the tool center point.
                                        Defaults to None in which case the last link is used.
        connector_frames (str, optional): The name of the urdf_link
                                            at which a robot connects.
                                            Defaults to None in which case the base link is used.
    """

    def __init__(self, urdf_model: str, start_position, start_orientation,
                 coupled_robots=None, tcp_frame=None, connector_frames=None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robots, tcp_frame, connector_frames)

        self._joint_name_to_index = {}
        self._lower_joint_limit = np.zeros(p.getNumJoints(self.urdf))
        self._upper_joint_limit = np.zeros(p.getNumJoints(self.urdf))
        self._actuated_joints = []

        for joint_number in range(p.getNumJoints(self.urdf)):
            if p.getJointInfo(self.urdf, joint_number)[2] != 4:
                joint_name = p.getJointInfo(self.urdf, joint_number)[
                    1].decode("utf-8")
                self._joint_name_to_index[joint_name] = joint_number
                self._actuated_joints.append(joint_number)

                lower_limit = p.getJointInfo(self.urdf, joint_number)[8]
                upper_limit = p.getJointInfo(self.urdf, joint_number)[9]
                if upper_limit < lower_limit:
                    lower_limit = -np.inf
                    upper_limit = np.inf
                self._lower_joint_limit[joint_number] = lower_limit
                self._upper_joint_limit[joint_number] = upper_limit

        self.max_joint_force = 20 * np.ones(p.getNumJoints(self.urdf))

        for joint_number in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

        self._gripper_constraints = []
        for i in range(1, len(self._actuated_joints)):
            self._gripper_constraints.append(p.createConstraint(self.urdf,
                                                                self._actuated_joints[0],
                                                                self.urdf,
                                                                self._actuated_joints[i],
                                                                jointType=p.JOINT_GEAR,
                                                                jointAxis=[
                                                                    1, 0, 0],
                                                                parentFramePosition=[
                                                                    0, 0, 0],
                                                                childFramePosition=[0, 0, 0]))

            p.changeConstraint(
                self._gripper_constraints[-1], gearRatio=-1, erp=0.1, maxForce=5000)
        for joint_number in self._actuated_joints:
            p.setJointMotorControl2(self.urdf, joint_number, p.VELOCITY_CONTROL, targetVelocity=0,
                                    force=self.max_joint_force[joint_number])

    def actuate(self, target: float):
        """Actuates the gripper.

        Args:
            target(float): Relative amount of maximum travel distance the gripper should move
         """
        for joint_number in self._actuated_joints:
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                    force=self.max_joint_force[joint_number],
                                    targetPosition=self._lower_joint_limit[joint_number] + (
                                        self._upper_joint_limit[joint_number]
                                        - self._lower_joint_limit[joint_number]) * target)


class SuctionGripper(EndeffectorTool):
    """The base class for all Tools and Sensors connected to a Robot

    Args:
        urdf_model (str): A valid path to a urdf file describing the tool geometry
        start_position (np.array): the position at which the tool should be spawned
        start_orientation (np.array): the orientation at which the tool should be spawned
        coupled_robots (pi.RobotBase, optional): A wbk_sim.Robot object if
                                                 the robot is coupled from the start.
                                                 Defaults to None.
        tcp_frame (str, optional): The name of the urdf_link
                                   describing the tool center point.
                                   Defaults to None in which case the last link is used.
        connector_frames (str, optional): The name of the urdf_link
                                          at which a robot connects.
                                          Defaults to None in which case the base link is used.
        suction_links (str, optional): The names of the urdf_links wich represent
                                       the active suction parts.
                                       Defaults to all Links.

    """

    def __init__(self, urdf_model: str, start_position, start_orientation,
                 coupled_robots=None, tcp_frame=None, connector_frames=None, suction_links=None):

        super().__init__(urdf_model, start_position, start_orientation,
                         coupled_robots, tcp_frame, connector_frames)
        self.suction_constraints = []
        self._suction_links_ids = []

        self._link_name_to_index = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf, joint_number)[
                12].decode("utf-8")
            self._link_name_to_index[link_name] = joint_number

        if suction_links is None:
            self._suction_links_ids = None
        else:
            for name in suction_links:
                self._suction_links_ids.append(self._link_name_to_index[name])

    def activate(self, tolerance=0.0001):
        """Function to activate the suction gripper creating constraints between gripper and object

        Args:
            tolerance (float, optional): tolerance of contacts, i.e.
                                         at which distance the contact point is considered relevant.
        Returns:
            list[int]: ids of coupled objects
        """
        contact_points = list(p.getContactPoints(self.urdf))
        position_g, orientation_g = self.get_tool_pose()
        inv_pos, inv_orn = p.invertTransform(position_g, orientation_g)

        coupled_bodys = []

        for point in contact_points:
            if point[2] is self.urdf or point[2] is self._coupled_robot.urdf or \
                    point[8] > tolerance:
                del point

        if self._suction_links_ids is not None:
            for point in contact_points:
                if point[3] not in self._suction_links_ids:
                    del point

        for point in contact_points:
            if point[2] not in coupled_bodys:
                coupled_bodys.append(point[2])

        for body in coupled_bodys:
            position_o, orientation_o = p.getBasePositionAndOrientation(body)
            restraint_pos, restraint_orn = p.multiplyTransforms(
                inv_pos, inv_orn, position_o, orientation_o)
            cid_grip = p.createConstraint(self.urdf, self._tcp_id,
                                          body, -1,
                                          p.JOINT_FIXED, [0, 0, 0],
                                          parentFramePosition=restraint_pos,
                                          childFramePosition=[0, 0, 0],
                                          parentFrameOrientation=restraint_orn,
                                          childFrameOrientation=None)
            self.suction_constraints.append(cid_grip)
        return coupled_bodys

    def deactivate(self):
        """Function to deactivate the suction gripper
           deleting constraints between gripper and object
        """
        for constraint in self.suction_constraints:
            p.removeConstraint(constraint)
