from typing import Dict

import numpy as np
import pybullet as p



class RobotBase:

    def __init__(self, urdf_model, start_position, start_orientation):
        urdf_flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.urdf = p.loadURDF(urdf_model,
                               start_position, start_orientation,
                               flags=urdf_flags,
                               useFixedBase=False)

        self._joint_state_shape = self.get_joint_state()
        self._joint_mappings = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            name = p.getJointInfo(self.urdf,joint_number)[1].decode("utf-8")#convert byte string to string
            self._joint_mappings[name]=joint_number
        
        self.max_joint_force = 800*np.ones(p.getNumJoints(self.urdf))
        for joint_number in range(p.getNumJoints(self.urdf)):
            p.resetJointState(self.urdf, joint_number, targetValue=0)

    def get_joint_state(self):
        """Returns the position of each joint as a dictionary keyed with their name

        Returns:
             Dict[str,Dict[str,float]]: The state of all joinst

        """
        joint_state = {}
        for joint_number in range(p.getNumJoints(self.urdf)):
            if p.getJointInfo(self.urdf,joint_number)[2] is not p.JOINT_FIXED:
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
                joint_number = self._joint_mappings[joint]
                p.setJointMotorControl2(self.urdf, joint_number, p.POSITION_CONTROL,
                                            force=self.max_joint_force[joint_number],
                                            targetPosition=joint_position)
        else:
            print([key in self._joint_state_shape.keys() for key in target.keys()])
            raise KeyError('Error: One or more joints are not part of the robot. ' +
                             'correct keys are: '+str(self._joint_state_shape.keys()))


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

    p.setRealTimeSimulation(1)
    for i in range(100):
        print(robot.get_joint_state())
        target_state = {'q1':i/100.0,'q2':i/50,'q3':i/20,'q4':i/10,'q5':i/5,'q6':i}
        robot.set_joint_position(target_state)
        time.sleep(0.1)

