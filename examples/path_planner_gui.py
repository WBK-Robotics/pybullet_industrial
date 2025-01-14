import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np


class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints and execute paths.
    """

    def __init__(self, root, robot: pi.RobotBase, path_planner: pi.PathPlanner, collision_checker: pi.CollisionChecker):
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
        self.start = self.robot.get_joint_state()  # Start configuration
        self.goal = self.robot.get_joint_state()  # Goal configuration
        self.joint_order = self.robot.get_moveable_joints()[0]  # Order of movable joints
        self.joint_limits = self.robot.get_joint_limits()  # Joint limits as dictionaries
        self.root.title("PyBullet Path Planning")
        self.joint_values = [tk.StringVar(value="0") for _ in range(len(self.joint_order))]  # Initialize joint values
        self.create_widgets()

    def create_widgets(self):
        """
        Creates and arranges the widgets in the GUI.
        """
        # Create the layout for the joints
        for i, joint_name in enumerate(self.joint_order):
            # Label for the joint
            tk.Label(self.root, text=f"{joint_name}").grid(row=i, column=0, padx=5, pady=5)

            # "+" button for incrementing joint value
            tk.Button(self.root, text="+", command=lambda i=i: self.increment_joint(i)).grid(
                row=i, column=1, padx=5, pady=5)

            # "-" button for decrementing joint value
            tk.Button(self.root, text="-", command=lambda i=i: self.decrement_joint(i)).grid(
                row=i, column=2, padx=5, pady=5)

            # Text field for displaying the joint value
            tk.Entry(self.root, textvariable=self.joint_values[i], width=10).grid(
                row=i, column=3, padx=5, pady=5)

        # Add Control Buttons
        tk.Button(self.root, text="Set as Start", command=self.set_as_start).grid(
            row=len(self.joint_order), column=0, pady=10)

        tk.Button(self.root, text="Set as End", command=self.set_as_goal).grid(
            row=len(self.joint_order), column=1, pady=10)

        tk.Button(self.root, text="Set Allowed Collision", command=self.set_allowed_collision).grid(
            row=len(self.joint_order), column=2, pady=10)

        tk.Button(self.root, text="Plan and Execute", command=self.plan_and_execute).grid(
            row=len(self.joint_order), column=3, pady=10)

        # Add "Update Joint Positions" button
        tk.Button(self.root, text="Update Joint Positions", command=self.update_joint_positions).grid(
            row=len(self.joint_order) + 1, column=0, columnspan=4, pady=10)

        # Exit Button
        tk.Button(self.root, text="Exit", command=self.root.quit).grid(
            row=len(self.joint_order) + 2, column=0, columnspan=4, pady=20)

    def update_joint_positions(self):
        """
        Updates the joint positions displayed in the text fields based on the robot's current state.
        """
        joint_state = self.robot.get_joint_state()
        for i, joint_name in enumerate(self.joint_order):
            position = joint_state[joint_name]['position']
            self.joint_values[i].set(f"{position:.2f}")
        print("Joint positions updated.")

    def increment_joint(self, index):
        """
        Increments the joint value and updates the robot.

        Args:
            index (int): Index of the joint to increment.
        """
        current_value = float(self.joint_values[index].get())
        new_value = current_value + 0.2

        # Ensure the new value is within limits
        joint_name = self.joint_order[index]
        if new_value <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_value:.2f}")
            self.apply_joint_position(index, new_value)

    def decrement_joint(self, index):
        """
        Decrements the joint value and updates the robot.

        Args:
            index (int): Index of the joint to decrement.
        """
        current_value = float(self.joint_values[index].get())
        new_value = current_value - 0.2

        # Ensure the new value is within limits
        joint_name = self.joint_order[index]
        if new_value >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_value:.2f}")
            self.apply_joint_position(index, new_value)

    def apply_joint_position(self, index, position):
        """
        Updates the robot's joint position in real-time.

        Args:
            index (int): Index of the joint to update.
            position (float): The new joint position.
        """
        joint_name = self.joint_order[index]
        target = {joint_name: position}
        self.robot.reset_joint_position(target)

    def set_as_start(self):
        """
        Sets the current joint positions as the start configuration.
        """
        self.start = {joint_name: float(self.joint_values[i].get()) for i, joint_name in enumerate(self.joint_order)}
        print("Start configuration set:", self.start)

    def set_as_goal(self):
        """
        Sets the current joint positions as the goal configuration.
        """
        self.goal = {joint_name: float(self.joint_values[i].get()) for i, joint_name in enumerate(self.joint_order)}
        print("Goal configuration set:", self.goal)

    def set_allowed_collision(self):
        """
        Configures allowed collisions for the robot.
        """
        self.collision_checker.allow_collision_links = self.collision_checker.get_collision_links()
        self.collision_checker.update_collision_settings()
        print("Allowed collisions set.")

    def plan_and_execute(self):
        """
        Plans and executes the robot's path based on the current start and goal positions.
        """
        print("Planning and executing path...")

        # Plan the path
        res, joint_path = self.path_planner.plan_start_goal(self.start, self.goal)
        if res:
            for joint_configuration, tool_activation in joint_path:
                self.robot.reset_joint_position(joint_configuration)
                time.sleep(0.01)

            print("Path execution completed.")
        else:
            print("No solution found.")
