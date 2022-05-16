import pybullet as p

class EndeffectorTool:
    def __init__(self,urdf_model: str, start_position, start_orientation,coupled_robot=None,tcp_frame=None):
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

        self._coupled_robot = coupled_robot
        self._coupling_constraint = p.createConstraint(self.urdf,
                                                        -1, -1, -1,
                                                        p.JOINT_FIXED,
                                                        [0, 0, 0],
                                                        [0, 0, 0],
                                                        start_position,
                                                        start_orientation)

        self._link_name_to_index = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf,joint_number)[12].decode("utf-8")
            self._link_name_to_index[link_name]=joint_number
        if tcp_frame == None:
            last_link = max(self._link_name_to_index)
            self._tcp_id = self._link_name_to_index[last_link]
        else:
            self._tcp_id = self._convert_endeffector(tcp_frame)

    def couple(self,robot,endeffector_name=None):
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
        #creates a fixed constrained between the choosen robot and the tool
        if self._coupled_robot is not None:
            raise ValueError("The Tool is already coupled with a robot")
        else:
            if endeffector_name == None:
                endeffector_index = robot._default_endeffector_id
            else:
                endeffector_index = robot._convert_endeffector(endeffector_name)
            self._coupled_robot = robot

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
         #converts between the robots forward kinematics and the tool forward kinematics
        pass

    def set_tool_pose(self,target_position,target_orientation=None,tcp_frame=None):
        #converts between the robots inverse kinematics and the tool inverse kinematics
        # Assumes the currently connected endeffector for the robot inv kin
        pass

