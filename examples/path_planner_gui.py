import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np
import sys

JOINT_INCREMENT: float = 0.1


class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints,
    update obstacles, and execute paths. Displays collision and
    constraint status.
    """

    def __init__(self, root: tk.Tk, robot: pi.RobotBase,
                 path_planner: pi.PathPlanner,
                 collision_check: list,
                 obstacle,
                 constraint_functions: list,
                 endeffector=None,
                 moved_object=None,
                 collision_checker=None) -> None:
        """
        Initializes the PathPlanner GUI.

        Args:
            root: The root Tkinter window.
            robot: The robot instance from PyBullet Industrial.
            path_planner: The PathPlanner instance for motion planning.
            collision_check: List of collision-check functions.
            obstacle: The obstacle to manipulate.
            constraint_functions: List of no-arg lambdas that return a
                boolean indicating if a constraint is satisfied.
            endeffector: Optional endeffector instance.
            moved_object: Optional moved object instance.
            collision_checker: Optional collision checker.
        """
        self.root: tk.Tk = root
        self.robot: pi.RobotBase = robot
        self.path_planner: pi.PathPlanner = path_planner
        self.collision_check: list = collision_check
        self.obstacle = obstacle
        self.constraint_functions: list = constraint_functions
        self.collision_checker = collision_checker
        # Get initial start and goal configurations.
        self.start: dict = self.robot.get_joint_state()
        self.goal: dict = self.robot.get_joint_state()
        self.joint_order: list = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")
        # Create StringVar list for joint values.
        self.joint_values: list = [tk.StringVar(value="0")
                                   for _ in range(len(self.joint_order))]
        # Create StringVar list for obstacle values (pos & orient).
        self.obstacle_values: list = [tk.StringVar(value="0")
                                      for _ in range(6)]
        # Status indicator for collision.
        self.collision_status: tk.StringVar = tk.StringVar(value="green")
        # Status indicator for constraint functions.
        self.constraint_status: tk.StringVar = tk.StringVar(value="green")
        # Initial obstacle box size.
        self.current_box_size: list = [0.5, 0.5, 0.05]
        self.endeffector = endeffector
        self.moved_object = moved_object
        self.create_widgets()
        self.update_joint_positions()
        self.set_initial_obstacle_values()
        self.update_status()

    def create_widgets(self) -> None:
        """
        Creates and arranges the widgets in the GUI.
        """
        # Create layout for each robot joint.
        for i, joint_name in enumerate(self.joint_order):
            tk.Label(self.root, text=f"{joint_name}").grid(
                row=i, column=0, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_joint(i)
                      ).grid(row=i, column=1, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_joint(i)
                      ).grid(row=i, column=2, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.joint_values[i],
                     width=10).grid(row=i, column=3, padx=5, pady=5)

        # Create layout for obstacle position and orientation.
        obstacle_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obstacle_labels):
            tk.Label(self.root, text=f"{label}").grid(
                row=i, column=4, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_obstacle(i)
                      ).grid(row=i, column=5, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_obstacle(i)
                      ).grid(row=i, column=6, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.obstacle_values[i],
                     width=10).grid(row=i, column=7, padx=5, pady=5)

        # Create status indicators for collision and constraints.
        status_row: int = len(self.joint_order)
        tk.Label(self.root, text="Collision Status:").grid(
            row=status_row, column=0, padx=5, pady=10)
        self.collision_light = tk.Label(
            self.root,
            bg=self.collision_status.get(),
            width=10, height=2)
        self.collision_light.grid(row=status_row, column=1,
                                  padx=5, pady=10)
        tk.Label(self.root, text="Constraint Status:").grid(
            row=status_row, column=2, padx=5, pady=10)
        self.constraint_light = tk.Label(
            self.root,
            bg=self.constraint_status.get(),
            width=10, height=2)
        self.constraint_light.grid(row=status_row, column=3,
                                   padx=5, pady=10)

        # Create control buttons.
        control_row: int = status_row + 1
        tk.Button(self.root, text="Set as Start",
                  command=self.set_as_start).grid(
                      row=control_row, column=0, pady=10)
        tk.Button(self.root, text="Set as End",
                  command=self.set_as_goal).grid(
                      row=control_row, column=1, pady=10)
        tk.Button(self.root, text="Plan and Execute",
                  command=self.plan_and_execute).grid(
                      row=control_row, column=3, pady=10)
        # Obstacle size control buttons.
        tk.Button(self.root, text="Shrink Obstacle",
                  command=self.shrink_obstacle).grid(
                      row=control_row + 1, column=4, pady=10)
        tk.Button(self.root, text="Grow Obstacle",
                  command=self.grow_obstacle).grid(
                      row=control_row + 1, column=5, pady=10)
        # Exit button.
        tk.Button(self.root, text="Exit",
                  command=self.root.quit).grid(
                      row=control_row + 2, column=0, columnspan=4,
                      pady=20)

    def update_status(self) -> None:
        """
        Updates the collision and constraint status indicators.
        """
        # Update endeffector and moved object if available.
        if self.endeffector:
            self.endeffector.match_endeffector_pose(self.robot)
            if self.moved_object:
                self.endeffector.match_moving_object(self.moved_object)
        # Check collision status.
        valid_collision: bool = all(cc() for cc in self.collision_check)
        self.collision_status.set("green" if valid_collision else "red")
        self.collision_light.config(bg=self.collision_status.get())
        # Update constraint status.
        self.update_constraint_status()

    def update_constraint_status(self) -> None:
        """
        Checks all constraint functions and updates the indicator.
        """
        valid_constraints: bool = all(fn() for fn in
                                      self.constraint_functions)
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def shrink_obstacle(self) -> None:
        """
        Shrinks the obstacle size by 20% and updates its display.
        """
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        # Scale down each box extent.
        self.current_box_size = [
            extent - (extent * 0.2) for extent in self.current_box_size
        ]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def grow_obstacle(self) -> None:
        """
        Grows the obstacle size by 20% and updates its display.
        """
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        # Increase each box extent.
        self.current_box_size = [
            extent + (extent * 0.2) for extent in self.current_box_size
        ]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def set_initial_obstacle_values(self) -> None:
        """
        Sets the GUI entries for the obstacle's initial position
        and orientation.
        """
        pos, orn_q = p.getBasePositionAndOrientation(self.obstacle)
        orn_e = p.getEulerFromQuaternion(orn_q)
        # Set position values.
        for i in range(3):
            self.obstacle_values[i].set(f"{pos[i]:.2f}")
        # Set orientation values.
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orn_e[i-3]:.2f}")

    def update_joint_positions(self) -> None:
        """
        Retrieves current joint positions from the robot and updates
        the GUI.
        """
        joint_state = self.robot.get_joint_state()
        for i, jn in enumerate(self.joint_order):
            pos = joint_state[jn]['position']
            self.joint_values[i].set(f"{pos:.2f}")
        self.update_status()

    def increment_joint(self, index: int) -> None:
        """
        Increments the joint value at the specified index.
        """
        cur: float = float(self.joint_values[index].get())
        new_val: float = cur + JOINT_INCREMENT
        joint_name: str = self.joint_order[index]
        # Ensure new value does not exceed upper limit.
        if new_val <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def decrement_joint(self, index: int) -> None:
        """
        Decrements the joint value at the specified index.
        """
        cur: float = float(self.joint_values[index].get())
        new_val: float = cur - JOINT_INCREMENT
        joint_name: str = self.joint_order[index]
        # Ensure new value is above lower limit.
        if new_val >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def increment_obstacle(self, index: int) -> None:
        """
        Increments the obstacle parameter at the given index.
        """
        cur: float = float(self.obstacle_values[index].get())
        new_val: float = cur + 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index: int) -> None:
        """
        Decrements the obstacle parameter at the given index.
        """
        cur: float = float(self.obstacle_values[index].get())
        new_val: float = cur - 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def update_obstacle(self) -> None:
        """
        Updates the obstacle's position and orientation based on the
        GUI values.
        """
        pos: list = [float(self.obstacle_values[i].get())
                     for i in range(3)]
        orn_e: list = [float(self.obstacle_values[i].get())
                       for i in range(3, 6)]
        orn_q = p.getQuaternionFromEuler(orn_e)
        p.resetBasePositionAndOrientation(self.obstacle, pos, orn_q)
        self.update_status()

    def apply_joint_position(self, index: int, position: float) -> None:
        """
        Applies the joint position change to the robot and updates status.
        """
        joint_name: str = self.joint_order[index]
        target: dict = {joint_name: position}
        self.robot.reset_joint_position(target)
        self.update_status()

    def set_as_start(self) -> None:
        """
        Sets the current joint configuration as the start configuration.
        """
        self.start = {jn: float(self.joint_values[i].get())
                      for i, jn in enumerate(self.joint_order)}
        print("Start configuration set:", self.start)

    def set_as_goal(self) -> None:
        """
        Sets the current joint configuration as the goal configuration.
        """
        self.goal = {jn: float(self.joint_values[i].get())
                     for i, jn in enumerate(self.joint_order)}
        print("Goal configuration set:", self.goal)

    def plan_and_execute(self) -> None:
        """
        Plans a path from the start to goal configuration and executes it.
        """
        print("Planning and executing path...")
        res, joint_path = self.path_planner.plan_start_goal(
            self.start, self.goal)
        if res:
            # Execute each joint configuration along the path.
            for joint_conf, tool_act in joint_path:
                self.robot.reset_joint_position(joint_conf, True)
                if self.endeffector:
                    self.endeffector.match_endeffector_pose(self.robot)
                    if self.moved_object:
                        self.endeffector.match_moving_object(
                            self.moved_object)
                time.sleep(0.01)
                # Check for collision during execution.
                if not self.collision_check[0]():
                    print("Collision detected!")
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
        self.update_status()
