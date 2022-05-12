from typing import Dict

import numpy as np
import pybullet as p






class RobotBase:

    def __init__(self, urdf_model, start_position, start_orientation):
        """AI is creating summary for __init__

        Args:
            urdf_model ([type]): [description]
            start_position ([type]): [description]
            start_orientation ([type]): [description]
        """
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._joint_state_shape = self.get_joint_state()
        self._joint_name_to_index = {}
        self._link_name_to_index = {}
        kinematic_solver_map = []
        self._lower_joint_limit = np.zeros(p.getNumJoints(self.urdf))
        self._upper_joint_limit = np.zeros(p.getNumJoints(self.urdf))

        for joint_number in range(p.getNumJoints(self.urdf)):
            link_name = p.getJointInfo(self.urdf,joint_number)[12].decode("utf-8")
            self._link_name_to_index[link_name]=joint_number

            if p.getJointInfo(self.urdf,joint_number)[2] is not 4:
                joint_name = p.getJointInfo(self.urdf,joint_number)[1].decode("utf-8")
                self._joint_name_to_index[joint_name]=joint_number

                kinematic_solver_map.append(joint_number)

                lower_limit = p.getJointInfo(self.urdf, joint_number)[8]
                upper_limit = p.getJointInfo(self.urdf, joint_number)[9]
                if upper_limit < lower_limit:
                    lower_limit = -np.inf
                    upper_limit = np.inf
                self._lower_joint_limit[joint_number] = lower_limit
                self._upper_joint_limit[joint_number] = upper_limit
        
        self._kinematic_solver_map = np.array(kinematic_solver_map)

        self.max_joint_force = 1000*np.ones(p.getNumJoints(self.urdf))
        for joint_number in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

        self._rooting_constraint = p.createConstraint(self.urdf,
                                                        -1, -1, -1,
                                                        p.JOINT_FIXED,
                                                        [0, 0, 0],
                                                        [0, 0, 0],
                                                        start_position,
                                                        start_orientation)

    def get_joint_state(self):
        """Returns the position of each joint as a dictionary keyed with their name

        Returns:
             Dict[str,Dict[str,float]]: The state of all joinst

        """
        joint_state = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            if p.getJointInfo(self.urdf,joint_number)[2] is not 4:
                joint = p.getJointInfo(self.urdf,joint_number)[1].decode("utf-8")#convert byte string to string
                joint_position = p.getJointState(self.urdf, joint_number)[0]
                joint_velocity = p.getJointState(self.urdf, joint_number)[1]
                joint_torque = p.getJointState(self.urdf, joint_number)[3]
                joint_reaction_force = p.getJointState(self.urdf, joint_number)[2]

                single_joint_state = {'position':joint_position,
                                    'velocity':joint_velocity,
                                    'torque':joint_torque,
                                    'reaction force':joint_reaction_force}
                joint_state[joint] = single_joint_state
        return joint_state

    def set_joint_position(self,target: Dict[str,  float]):
        """Sets the target position for a number of joints.
           The maximum force of each joint is set according to the max_joint_force class attribute.

        Args:
            target (Dict[str,  float]): A dictionary containing the joint states to be set

        Raises:
            KeyError: If the specified joint state is not part of the Robot
        """
        if all(key in self._joint_state_shape.keys() for key in target.keys()):
            for joint, joint_position in target.items():
                joint_number = self._joint_name_to_index[joint]

                lower_joint_limit = self._lower_joint_limit[joint_number]
                upper_joint_limit = self._upper_joint_limit[joint_number]
                if joint_position > upper_joint_limit or joint_position < lower_joint_limit:
                    raise ValueError('The joint position '+str(joint_position)+
                                      ' is aut of limit for joint '+joint+'. Its limits are:\n'+
                                      str(lower_joint_limit)+' and '+str(upper_joint_limit))

                p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                            force=self.max_joint_force[joint_number],
                                            targetPosition=joint_position)
        else:
            raise KeyError('One or more joints are not part of the robot. ' +
                             'correct keys are: '+str(self._joint_state_shape.keys()))


    def get_endeffector_pose(self,endeffector):
        """Returns the position of the endeffector in world coordinates

        Args:
            endeffector (str): The name of the endeffector link

        Returns:
            array: The position of the endeffector
            array: The orientation of the endeffector as a quaternion
        """
        endeffector_id = self._link_name_to_index[endeffector]
        link_state = p.getLinkState(self.urdf,endeffector_id)

        position = np.array(link_state[0])
        orientation = np.array(link_state[1])
        return position, orientation

    def set_endeffector_pose(self,endeffector,target_position,target_orientation=None):
        endeffector_id = self._link_name_to_index[endeffector]

        if target_orientation is None:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                    endeffector_id,
                                                    target_position,
                                                    lowerLimits = self._lower_joint_limit,
                                                    upperLimits = self._upper_joint_limit)
        else:
            joint_poses = p.calculateInverseKinematics(self.urdf,
                                                    endeffector_id,
                                                    target_position,
                                                    targetOrientation = target_orientation,
                                                    lowerLimits = self._lower_joint_limit,
                                                    upperLimits = self._upper_joint_limit)


    
        for index, joint_position in enumerate(joint_poses):
            joint_number = self._kinematic_solver_map[index]
            p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                            force=self.max_joint_force[joint_number],
                                            targetPosition=joint_position)

    def reset_robot(self, start_position, start_orientation, joint_values=None):
        """resets the robots joints to 0 and the base to a specified position and orientation

        Args:
            start_position ([type]): a 3 dimensional position
            start_orientation ([type]): a 4 dimensional quaternion representing
                                       the desired orientation
        """
        self.set_world_state(start_position, start_orientation)

        if joint_values is None:
            joint_values = np.zeros(p.getNumJoints(self.urdf))
        for joint in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint,
                              targetValue=joint_values[joint])



    def set_world_state(self, start_position, start_orientation):
        """Resets the robots base to a specified position and orientation

        Args:
            start_position ([type]): a 3 dimensional position
            start_orientation ([type]): a 4 dimensional quaternion representing
                                       the desired orientation
        """
        p.changeConstraint(self._rooting_constraint,start_position,start_orientation)
        p.resetBasePositionAndOrientation(
            self.urdf, start_position, start_orientation)

    def get_world_state(self):
        """Returns the position and orientation of the robot relative to the world

        Returns:
            [type]: a 3 dimensional position and a 4 dimensional quaternion representing
                                       the current orientation
        """
        return p.getBasePositionAndOrientation(self.urdf)


if __name__ == "__main__":
    import os
    import time
    dirname = os.path.dirname(__file__)
    parentDir = os.path.dirname(dirname)
    urdf_file1 = os.path.join(
            dirname, 'robot_descriptions', 'comau_NJ290_3-0_m.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=1000)
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = RobotBase(urdf_file1,[0,0,0],start_orientation)

    
    p.createConstraint(robot.urdf,
                       -1, -1, -1,
                       p.JOINT_FIXED,
                       [0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 0])
    

    p.setRealTimeSimulation(1)
    print(robot._joint_name_to_index)
    for i in range(1000): 
        target_state = {'q1':i/100.0,'q2':i/50,'q3':i/20,'q4':i/10,'q5':i/5,'q6':i} 
        #robot.set_joint_position(target_state) 
        target_orientation = p.getQuaternionFromEuler([0, i/100, 0]) 
        target_pose = [1.9,0,1.2] 
        robot.set_endeffector_pose('link6',target_pose,target_orientation,iterations=1) 
        #print(robot._joint_name_to_index)
        print(robot.get_endeffector_pose('link6')) 
        #print(robot.get_joint_state())
        time.sleep(0.1) 