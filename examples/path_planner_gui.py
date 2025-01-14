import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np

class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints and execute paths.
    """

    def __init__(self, root, robot: pi.RobotBase, path_planner: pi.PathPlanner, collision_checker: pi.CollisionChecker, start, goal):
        """
        Initializes the PathPlanner GUI.

        Args:
            root (tk.Tk): The root Tkinter window.
            robot: The robot instance from PyBullet Industrial.
            path_planner: The PathPlanner instance for motion planning.
            collision_checker: The CollisionChecker instance for collision management.
            start (dict): The starting joint positions.
            goal (dict): The goal joint positions.
        """
        self.root = root
        self.robot = robot
        self.path_planner = path_planner
        self.collision_checker = collision_checker
        self.start = start
        self.goal = goal
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")
        self.joint_values = [tk.StringVar(value="0") for _ in range(6)]  # Initialize joint values
        self.create_widgets()

    def create_widgets(self):
        """
        Creates and arranges the widgets in the GUI.
        """
        # Create the layout for the joints
        for i in range(6):
            # Label for the joint
            tk.Label(self.root, text=f"Joint {i + 1}").grid(row=i, column=0, padx=5, pady=5)

            # "+" button for incrementing joint value
            tk.Button(self.root, text="+", command=lambda i=i: self.increment_joint(i)).grid(
                row=i, column=1, padx=5, pady=5)

            # "-" button for decrementing joint value
            tk.Button(self.root, text="-", command=lambda i=i: self.decrement_joint(i)).grid(
                row=i, column=2, padx=5, pady=5)

            # Text field for displaying the joint value
            tk.Entry(self.root, textvariable=self.joint_values[i], width=10).grid(
                row=i, column=3, padx=5, pady=5)

        # Plan and Execute Button
        tk.Button(self.root, text="Plan and Execute Path", command=self.plan_and_execute).grid(
            row=6, column=0, columnspan=2, pady=20)

        # Exit Button
        tk.Button(self.root, text="Exit", command=self.root.quit).grid(
            row=6, column=2, columnspan=2, pady=20)

    def get_joint_position(self):
        """
        Retrieves the current joint positions of the robot.

        Returns:
            np.array: An array containing the joint positions in the order of `joint_order`.
        """
        joint_order = self.robot.get_moveable_joints()[0]  # Retrieve the joint order
        joint_state = self.robot.get_joint_state()  # Get the full joint state dictionary

        # Retrieve positions in the correct order
        joint_positions = [
            joint_state[joint_name]['position'] for joint_name in joint_order
        ]

        return pi.JointPath(np.array([joint_positions]).transpose(), joint_order)

    def set_joint_position(self, joint_position: pi.ToolPath):
        for joint_configuration, _ in joint_position:
            self.robot.reset_joint_position(joint_configuration)
            time.sleep(0.01)

    def uppdate_joint_position(self):
        joint_state = self.robot.get_joint_state()
        joint_positions = []

        for joint_name in self.robot.get_moveable_joints()[0]:
            joint_positions.append(joint_state[joint_name]['position'])


    def increment_joint(self, index):
        """
        Increments the joint value.

        Args:
            index (int): Index of the joint to increment.
        """
        current_value = int(self.joint_values[index].get())
        self.joint_values[index].set(current_value + 1)

    def decrement_joint(self, index):
        """
        Decrements the joint value.

        Args:
            index (int): Index of the joint to decrement.
        """
        current_value = int(self.joint_values[index].get())
        self.joint_values[index].set(current_value - 1)

    def plan_and_execute(self):
        """
        Plans and executes the robot's path based on the current start and goal positions.
        """
        print("Button clicked. Planning and executing path...")

        # Update collision settings
        self.collision_checker.allow_collision_links = self.collision_checker.get_collision_links()
        self.collision_checker.update_collision_settings()

        # Plan the path
        res, joint_path = self.path_planner.plan_start_goal(self.start, self.goal)
        if res:
            for joint_configuration, tool_activation in joint_path:
                self.robot.reset_joint_position(joint_configuration)
                time.sleep(0.01)
        else:
            print("No solution found.")
