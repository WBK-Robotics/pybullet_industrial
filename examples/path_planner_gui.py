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
    constraint status. Supports multiple planner instances via a
    dropdown menu.
    """

    def __init__(self, root: tk.Tk, path_planner, obstacle) -> None:
        """
        Initializes the PathPlanner GUI.

        Args:
            root: The root Tkinter window.
            path_planner: A single RobotPlannerSimpleSetup instance or a list
                of such instances.
            obstacle: The obstacle to manipulate.
        """
        self.root: tk.Tk = root

        # Support multiple planner instances.
        if isinstance(path_planner, list):
            self.path_planners = path_planner
            self.planner_mapping = {}
            for idx, planner in enumerate(self.path_planners):
                name = f"Planner {idx + 1}"
                self.planner_mapping[name] = planner
            self.selected_planner_var = tk.StringVar(
                value=list(self.planner_mapping.keys())[0])
            self.path_planner = self.planner_mapping[
                self.selected_planner_var.get()]
        else:
            self.path_planners = [path_planner]
            self.planner_mapping = {"Planner 1": path_planner}
            self.selected_planner_var = tk.StringVar(value="Planner 1")
            self.path_planner = path_planner

        # Retrieve robot and related objects from the planner instance.
        self.robot = self.path_planner.robot
        self.collision_check = self.path_planner.validity_checker.collision_check_functions
        self.constraint_functions = self.path_planner.validity_checker.constraint_functions
        self.object_mover = self.path_planner.space_information.object_mover

        self.obstacle = obstacle

        # Get initial start and goal configurations from the robot.
        self.start: dict = self.robot.get_joint_state()
        self.goal: dict = self.robot.get_joint_state()
        self.joint_order: list = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")

        # Create StringVars for joint values.
        self.joint_values: list = [tk.StringVar(value="0")
                                   for _ in range(len(self.joint_order))]
        # Create StringVars for obstacle position and orientation.
        self.obstacle_values: list = [tk.StringVar(value="0")
                                      for _ in range(6)]
        # Status indicators.
        self.collision_status: tk.StringVar = tk.StringVar(value="green")
        self.constraint_status: tk.StringVar = tk.StringVar(value="green")
        # Initial obstacle box size.
        self.current_box_size: list = [0.5, 0.5, 0.05]

        self.create_widgets()
        self.update_joint_positions()
        self.set_initial_obstacle_values()
        self.update_status()

    def create_widgets(self) -> None:
        """
        Creates and arranges the widgets in the GUI.
        """
        # Layout for each robot joint.
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

        # Layout for obstacle position and orientation.
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

        # Row for additional controls.
        base_row: int = len(self.joint_order)
        # Dropdown for selecting planner instance.
        tk.Label(self.root, text="Select Planner:").grid(
            row=base_row, column=0, padx=5, pady=5)
        planner_menu = tk.OptionMenu(
            self.root,
            self.selected_planner_var,
            *self.planner_mapping.keys(),
            command=self.update_selected_planner
        )
        planner_menu.grid(row=base_row, column=1, padx=5, pady=5)

        # Status indicators.
        status_row: int = base_row + 1
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

        # Control buttons.
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

    def update_selected_planner(self, selection: str) -> None:
        """
        Updates the active planner based on the dropdown selection.

        Args:
            selection (str): The name of the selected planner.
        """
        self.path_planner = self.planner_mapping[selection]
        # Update dependent objects from the newly selected planner.
        self.robot = self.path_planner.robot
        self.collision_check = self.path_planner.validity_checker.\
            collision_check_functions
        self.constraint_functions = (self.path_planner.validity_checker.\
            constraint_functions or [])
        self.object_mover = self.path_planner.space_information.object_mover
        print(f"Selected planner: {selection}")

    def update_status(self) -> None:
        """
        Updates collision and constraint status indicators.
        """
        if self.object_mover:
            pos, ori = self.robot.get_endeffector_pose()
            self.object_mover.match_moving_objects(
                pos, ori)
        valid_collision: bool = all(cc() for cc in self.collision_check)
        self.collision_status.set("green" if valid_collision else "red")
        self.collision_light.config(bg=self.collision_status.get())
        self.update_constraint_status()

    def update_constraint_status(self) -> None:
        """
        Checks all constraint functions and updates the indicator.
        """
        valid_constraints = True
        if self.constraint_functions:
            valid_constraints: bool = all(fn() for fn in self.constraint_functions)
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def shrink_obstacle(self) -> None:
        """
        Shrinks the obstacle size by 20% and updates its display.
        """
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
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
        Sets GUI entries for the obstacle's initial position and orientation.
        """
        pos, orn_q = p.getBasePositionAndOrientation(self.obstacle)
        orn_e = p.getEulerFromQuaternion(orn_q)
        for i in range(3):
            self.obstacle_values[i].set(f"{pos[i]:.2f}")
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orn_e[i-3]:.2f}")

    def update_joint_positions(self) -> None:
        """
        Retrieves current joint positions from the robot and updates the GUI.
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
        Updates the obstacle's position and orientation based on GUI values.
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
        Applies a joint position change to the robot and updates status.
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
        Plans a path from start to goal configuration and executes it.
        """
        print("Planning and executing path...")
        res, joint_path = self.path_planner.plan_start_goal(
            self.start, self.goal)
        if res:
            for joint_conf, tool_act in joint_path:
                self.robot.reset_joint_position(joint_conf, True)
                if self.object_mover:
                    pos, ori = self.robot.get_endeffector_pose()
                    self.object_mover.match_moving_objects(
                        pos, ori)
                time.sleep(0.005)
                # if not self.collision_check[0]():
                #     print("Collision detected!")
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
        self.update_status()
