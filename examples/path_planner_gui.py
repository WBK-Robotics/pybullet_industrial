import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np


JOINT_INCREMENT = 0.2


class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints and execute paths.
    """

    def __init__(self, root, robot: pi.RobotBase, path_planner: pi.PathPlanner, collision_checker: pi.CollisionChecker, obstacle):
        """
        Initializes the PathPlanner GUI.

        Args:
            root (tk.Tk): The root Tkinter window.
            robot: The robot instance from PyBullet Industrial.
            path_planner: The PathPlanner instance for motion planning.
            collision_checker: The CollisionChecker instance for collision management.
            obstacle: The obstacle to manipulate.
        """
        self.root = root
        self.robot = robot
        self.path_planner = path_planner
        self.collision_checker = collision_checker
        self.obstacle = obstacle
        self.start = self.robot.get_joint_state()  # Start configuration
        self.goal = self.robot.get_joint_state()  # Goal configuration
        self.joint_order = self.robot.get_moveable_joints()[0]  # Order of movable joints
        self.joint_limits = self.robot.get_joint_limits()  # Joint limits as dictionaries
        self.root.title("PyBullet Path Planning")
        self.joint_values = [tk.StringVar(value="0") for _ in range(len(self.joint_order))]  # Initialize joint values
        self.obstacle_values = [tk.StringVar(value="0") for _ in range(6)]  # Initialize obstacle values (X, Y, Z, A, B, C)
        self.light_color = tk.StringVar(value="green")  # Light color indicator
        self.current_box_size = [0.5, 0.5, 0.05]
        self.create_widgets()
        self.set_allowed_collision()
        self.update_joint_positions()
        self.set_initial_obstacle_values()

    def create_widgets(self):
        """
        Creates and arranges the widgets in the GUI.
        """
        # Create the layout for the robot's joints
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

        # Create the layout for the obstacle's position and orientation
        obstacle_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obstacle_labels):
            # Label for the obstacle axis
            tk.Label(self.root, text=f"{label}").grid(row=i, column=4, padx=5, pady=5)

            # "+" button for incrementing obstacle value
            tk.Button(self.root, text="+", command=lambda i=i: self.increment_obstacle(i)).grid(
                row=i, column=5, padx=5, pady=5)

            # "-" button for decrementing obstacle value
            tk.Button(self.root, text="-", command=lambda i=i: self.decrement_obstacle(i)).grid(
                row=i, column=6, padx=5, pady=5)

            # Text field for displaying the obstacle value
            tk.Entry(self.root, textvariable=self.obstacle_values[i], width=10).grid(
                row=i, column=7, padx=5, pady=5)

        # Collision Light Indicator
        tk.Label(self.root, text="Collision Status:").grid(row=len(self.joint_order), column=0, padx=5, pady=10)
        self.light_label = tk.Label(self.root, bg=self.light_color.get(), width=10, height=2)
        self.light_label.grid(row=len(self.joint_order), column=1, padx=5, pady=10)

        # Add Control Buttons
        tk.Button(self.root, text="Set as Start", command=self.set_as_start).grid(
            row=len(self.joint_order) + 1, column=0, pady=10)

        tk.Button(self.root, text="Set as End", command=self.set_as_goal).grid(
            row=len(self.joint_order) + 1, column=1, pady=10)

        tk.Button(self.root, text="Set Allowed Collision", command=self.set_allowed_collision).grid(
            row=len(self.joint_order) + 1, column=2, pady=10)

        tk.Button(self.root, text="Plan and Execute", command=self.plan_and_execute).grid(
            row=len(self.joint_order) + 1, column=3, pady=10)

        tk.Button(self.root, text="Update Joint Positions", command=self.update_joint_positions).grid(
            row=len(self.joint_order) + 2, column=0, columnspan=2, pady=10)

        # New Buttons for Obstacle Size
        tk.Button(self.root, text="Shrink Obstacle", command=self.shrink_obstacle).grid(
            row=len(self.joint_order) + 3, column=4, pady=10)

        tk.Button(self.root, text="Grow Obstacle", command=self.grow_obstacle).grid(
            row=len(self.joint_order) + 3, column=5, pady=10)

        # Exit Button
        tk.Button(self.root, text="Exit", command=self.root.quit).grid(
            row=len(self.joint_order) + 3, column=0, columnspan=4, pady=20)

        tk.Button(self.root, text="Check Endeffector", command=self.check_endeffector_upright).grid(
            row=len(self.joint_order) + 3, column=6, pady=20)
        tk.Button(self.root, text="Upright Endeffedtor", command=self.make_endeffector_upright).grid(
            row=len(self.joint_order) + 3, column=7, pady=20)

    def make_endeffector_upright(self):
        position = self.robot.get_endeffector_pose()[0]
        orientation = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
        self.robot.set_endeffector_pose(position, orientation)
        self.update_joint_positions()

    def check_endeffector_upright(self):
        """
        Checks if the end effector is upright.
        """
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        # Target orientation
        target_orientation = np.array([-np.pi / 2, 0, 0])

        # Tolerance of ±0.1
        tolerance = np.array([0.3, 0.3, 2*np.pi])

        # Current orientation
        if np.all(np.abs(orientation - target_orientation) <= tolerance):
            print("Upright")
            print(np.abs(orientation - target_orientation))
        else:
            print("Fail")
            print(np.abs(orientation - target_orientation))

    def shrink_obstacle(self):
        """
        Shrinks the obstacle by reducing its size in all dimensions.
        """
        current_position, current_orientation = p.getBasePositionAndOrientation(self.obstacle)

        # Reduce the size of the obstacle
        self.current_box_size = [(extent - (extent*0.2)) for extent in self.current_box_size]

        # Remove the current obstacle and recreate it with new dimensions
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=current_position,
            baseOrientation=current_orientation
        )
        self.set_initial_obstacle_values()
        self.update_light_status()

    def grow_obstacle(self):
        """
        Grows the obstacle by increasing its size in all dimensions.
        """
        current_position, current_orientation = p.getBasePositionAndOrientation(self.obstacle)

        # Increase the size of the obstacle
        self.current_box_size = [(extent + (extent*0.2)) for extent in self.current_box_size]

        # Remove the current obstacle and recreate it with new dimensions
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=current_position,
            baseOrientation=current_orientation
        )
        self.set_initial_obstacle_values()
        self.update_light_status()

    def set_initial_obstacle_values(self):
        """
        Sets the initial position and orientation of the obstacle into the text fields.
        """
        position, orientation_quat = p.getBasePositionAndOrientation(self.obstacle)
        orientation_euler = p.getEulerFromQuaternion(orientation_quat)

        # Update obstacle values
        for i in range(3):
            self.obstacle_values[i].set(f"{position[i]:.2f}")
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orientation_euler[i - 3]:.2f}")

    def update_light_status(self):
        """
        Updates the light color based on the collision status.
        """
        if self.path_planner.is_state_valid(None):
            self.light_color.set("green")
        else:
            self.light_color.set("red")
        self.light_label.config(bg=self.light_color.get())

    def update_joint_positions(self):
        """
        Updates the joint positions displayed in the text fields.
        """
        joint_state = self.robot.get_joint_state()
        for i, joint_name in enumerate(self.joint_order):
            position = joint_state[joint_name]['position']
            self.joint_values[i].set(f"{position:.2f}")
        self.update_light_status()

    def increment_joint(self, index):
        """
        Increments the joint value and updates the robot.

        Args:
            index (int): Index of the joint to increment.
        """
        current_value = float(self.joint_values[index].get())
        new_value = current_value + JOINT_INCREMENT

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
        new_value = current_value - JOINT_INCREMENT

        # Ensure the new value is within limits
        joint_name = self.joint_order[index]
        if new_value >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_value:.2f}")
            self.apply_joint_position(index, new_value)

    def increment_obstacle(self, index):
        """
        Increments the obstacle value and updates its position or orientation.

        Args:
            index (int): Index of the obstacle parameter to increment.
        """
        current_value = float(self.obstacle_values[index].get())
        new_value = current_value + 0.1
        self.obstacle_values[index].set(f"{new_value:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index):
        """
        Decrements the obstacle value and updates its position or orientation.

        Args:
            index (int): Index of the obstacle parameter to decrement.
        """
        current_value = float(self.obstacle_values[index].get())
        new_value = current_value - 0.1
        self.obstacle_values[index].set(f"{new_value:.2f}")
        self.update_obstacle()

    def update_obstacle(self):
        """
        Updates the obstacle's position and orientation in the simulation and
        refreshes the collision status light.
        """
        position = [float(self.obstacle_values[i].get()) for i in range(3)]
        orientation_euler = [float(self.obstacle_values[i].get()) for i in range(3, 6)]
        orientation_quat = p.getQuaternionFromEuler(orientation_euler)

        p.resetBasePositionAndOrientation(self.obstacle, position, orientation_quat)

        # Update the collision status light
        self.update_light_status()

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
        self.update_light_status()

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
                self.robot.reset_joint_position(joint_configuration, True)
                time.sleep(0.01)

            # Update joint positions after path execution
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
