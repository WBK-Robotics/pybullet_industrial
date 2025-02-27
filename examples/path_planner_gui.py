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
    constraint status. Supports multiple planner setups, planner types,
    and optimization objectives via dropdown menus.
    """

    def __init__(self, root: tk.Tk, planner_setup, obstacle, planner_list, objective_list) -> None:
        """
        Initializes the PathPlanner GUI.

        Args:
            root: The root Tkinter window.
            planner_setup: A single RobotPlannerSimpleSetup instance or a list
                of such instances.
            obstacle: The obstacle to manipulate.
            planner_list: List of planner types (classes) that can be set using
                planner_setup.setPlanner(planner_type)
            objective_list: List of optimization objective options. Each option
                is expected to be a list of (objective, weight) pairs or None.
        """
        self.root: tk.Tk = root

        # Support multiple planner setups.
        if isinstance(planner_setup, list):
            self.planner_setups = planner_setup
            self.planner_mappings = {}
            for planner in self.planner_setups:
                self.planner_mappings[planner.name] = planner
            self.selected_planner_var = tk.StringVar(
                value=list(self.planner_mappings.keys())[0])
            self.planner_setup = self.planner_mappings[
                self.selected_planner_var.get()]
        else:
            self.planner_setups = [planner_setup]
            self.planner_mappings = {planner_setup.name: planner_setup}
            self.selected_planner_var = tk.StringVar(value=planner_setup.name)
            self.planner_setup = planner_setup

        # Store planner types.
        self.planner_types = planner_list
        # Create a mapping from planner type names to planner classes.
        self.planner_type_mapping = {ptype.__name__: ptype for ptype in self.planner_types}
        self.selected_planner_type_var = tk.StringVar(
            value=list(self.planner_type_mapping.keys())[0])

        # Store optimization objective options.
        self.objective_options = objective_list
        # Build a mapping for displaying names.
        # If an option is None, we display "None". Otherwise, if it has a __name__,
        # we use that; if not, we use its string representation.
        self.objective_mapping = {}
        for obj in self.objective_options:
            if obj is None:
                key = "None"
            elif hasattr(obj, "__name__"):
                key = obj.__name__
            else:
                key = str(obj)
            self.objective_mapping[key] = obj
        self.selected_objective_var = tk.StringVar(
            value=list(self.objective_mapping.keys())[0])

        # Retrieve robot and related objects from the planner setup.
        self.robot = self.planner_setup.robot
        self.collision_check = self.planner_setup.validity_checker.collision_check_functions
        self.constraint_functions = self.planner_setup.validity_checker.constraint_functions
        self.object_mover = self.planner_setup.space_information.object_mover

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
        Creates and arranges the widgets in the GUI by grouping related
        controls into separate frames.
        """
        # Create a frame for robot joint controls.
        joints_frame = tk.LabelFrame(self.root, text="Robot Joints",
                                     padx=5, pady=5)
        joints_frame.grid(row=0, column=0, columnspan=4, sticky="nw",
                          padx=5, pady=5)

        for i, joint_name in enumerate(self.joint_order):
            tk.Label(joints_frame, text=joint_name).grid(
                row=i, column=0, padx=5, pady=5)
            tk.Button(joints_frame, text="+",
                      command=lambda i=i: self.increment_joint(i)
                      ).grid(row=i, column=1, padx=5, pady=5)
            tk.Button(joints_frame, text="-",
                      command=lambda i=i: self.decrement_joint(i)
                      ).grid(row=i, column=2, padx=5, pady=5)
            tk.Entry(joints_frame, textvariable=self.joint_values[i],
                     width=10).grid(row=i, column=3, padx=5, pady=5)

        # Create a frame for obstacle controls.
        obstacle_frame = tk.LabelFrame(self.root, text="Obstacle Controls",
                                       padx=5, pady=5)
        obstacle_frame.grid(row=0, column=4, columnspan=4, sticky="ne",
                            padx=5, pady=5)

        obstacle_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obstacle_labels):
            tk.Label(obstacle_frame, text=label).grid(
                row=i, column=0, padx=5, pady=5)
            tk.Button(obstacle_frame, text="+",
                      command=lambda i=i: self.increment_obstacle(i)
                      ).grid(row=i, column=1, padx=5, pady=5)
            tk.Button(obstacle_frame, text="-",
                      command=lambda i=i: self.decrement_obstacle(i)
                      ).grid(row=i, column=2, padx=5, pady=5)
            tk.Entry(obstacle_frame, textvariable=self.obstacle_values[i],
                     width=10).grid(row=i, column=3, padx=5, pady=5)

        # Create a frame for planner selection, type, objective dropdowns,
        # status indicators, and path controls.
        control_frame = tk.Frame(self.root, padx=5, pady=5)
        control_frame.grid(row=1, column=0, columnspan=8, sticky="ew",
                          padx=5, pady=5)

        # Planner setup selection dropdown.
        tk.Label(control_frame, text="Select Planner Setup:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w")
        planner_setup_menu = tk.OptionMenu(
            control_frame,
            self.selected_planner_var,
            *self.planner_mappings.keys(),
            command=self.update_selected_planner
        )
        planner_setup_menu.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        # Planner type selection dropdown.
        tk.Label(control_frame, text="Select Planner Type:").grid(
            row=0, column=2, padx=5, pady=5, sticky="w")
        planner_type_menu = tk.OptionMenu(
            control_frame,
            self.selected_planner_type_var,
            *self.planner_type_mapping.keys(),
            command=self.update_planner_type
        )
        planner_type_menu.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        # Optimization objective selection dropdown.
        tk.Label(control_frame, text="Select Optimization Objective:").grid(
            row=0, column=4, padx=5, pady=5, sticky="w")
        objective_menu = tk.OptionMenu(
            control_frame,
            self.selected_objective_var,
            *self.objective_mapping.keys(),
            command=self.update_objective
        )
        objective_menu.grid(row=0, column=5, padx=5, pady=5, sticky="w")

        # Status indicators.
        tk.Label(control_frame, text="Collision Status:").grid(
            row=1, column=0, padx=5, pady=10, sticky="w")
        self.collision_light = tk.Label(
            control_frame,
            bg=self.collision_status.get(),
            width=10, height=2
        )
        self.collision_light.grid(row=1, column=1, padx=5, pady=10, sticky="w")
        tk.Label(control_frame, text="Constraint Status:").grid(
            row=1, column=2, padx=5, pady=10, sticky="w")
        self.constraint_light = tk.Label(
            control_frame,
            bg=self.constraint_status.get(),
            width=10, height=2
        )
        self.constraint_light.grid(row=1, column=3, padx=5, pady=10, sticky="w")

        # Create a frame for path control buttons.
        path_controls_frame = tk.LabelFrame(
            control_frame, text="Path Controls", padx=5, pady=5)
        path_controls_frame.grid(row=2, column=0, columnspan=6, sticky="ew",
                                  padx=5, pady=10)

        tk.Button(path_controls_frame, text="Set as Start",
                  command=self.set_as_start).grid(
            row=0, column=0, padx=5, pady=10)
        tk.Button(path_controls_frame, text="Set as End",
                  command=self.set_as_goal).grid(
            row=0, column=1, padx=5, pady=10)
        tk.Button(path_controls_frame, text="Plan and Execute",
                  command=self.plan_and_execute).grid(
            row=0, column=2, padx=5, pady=10)
        tk.Button(path_controls_frame, text="Shrink Obstacle",
                  command=self.shrink_obstacle).grid(
            row=1, column=0, padx=5, pady=10)
        tk.Button(path_controls_frame, text="Grow Obstacle",
                  command=self.grow_obstacle).grid(
            row=1, column=1, padx=5, pady=10)

        tk.Button(control_frame, text="Exit",
                  command=self.root.quit).grid(
            row=3, column=0, columnspan=6, padx=5, pady=20)

    def update_selected_planner(self, selection: str) -> None:
        """
        Updates the active planner setup based on the dropdown selection.
        """
        self.planner_setup = self.planner_mappings[selection]
        self.robot = self.planner_setup.robot
        self.collision_check = self.planner_setup.validity_checker.collision_check_functions
        self.constraint_functions = (self.planner_setup.validity_checker.constraint_functions or [])
        self.object_mover = self.planner_setup.space_information.object_mover
        print(f"Selected planner setup: {selection}")

    def update_planner_type(self, selection: str) -> None:
        """
        Updates the planner type of the currently active planner setup.
        """
        planner_type = self.planner_type_mapping[selection]
        self.planner_setup.setPlanner(planner_type)
        print(f"Set planner type to: {selection}")

    def update_objective(self, selection: str) -> None:
        """
        Updates the optimization objective of the currently active planner setup.
        Calls planner_setup.setOptimizationObjective(objectives) with the
        selected objective option.
        """
        selected_objective = self.objective_mapping[selection]
        self.planner_setup.setOptimizationObjective(selected_objective)
        print(f"Set optimization objective to: {selection}")

    def update_status(self) -> None:
        """
        Updates collision and constraint status indicators.
        """
        if self.object_mover:
            pos, ori = self.robot.get_endeffector_pose()
            self.object_mover.match_moving_objects(pos, ori)
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
            valid_constraints = all(fn() for fn in self.constraint_functions)
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def shrink_obstacle(self) -> None:
        """
        Shrinks the obstacle size by 20% and updates its display.
        """
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [extent - (extent * 0.2) for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def grow_obstacle(self) -> None:
        """
        Grows the obstacle size by 20% and updates its display.
        """
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [extent + (extent * 0.2) for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=self.current_box_size),
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
        pos: list = [float(self.obstacle_values[i].get()) for i in range(3)]
        orn_e: list = [float(self.obstacle_values[i].get()) for i in range(3, 6)]
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
        self.start = {jn: float(self.joint_values[i].get()) for i, jn in enumerate(self.joint_order)}
        print("Start configuration set:", self.start)

    def set_as_goal(self) -> None:
        """
        Sets the current joint configuration as the goal configuration.
        """
        self.goal = {jn: float(self.joint_values[i].get()) for i, jn in enumerate(self.joint_order)}
        print("Goal configuration set:", self.goal)

    def plan_and_execute(self) -> None:
        """
        Plans a path from start to goal configuration and executes it.
        """
        print("Planning and executing path...")
        res, joint_path = self.planner_setup.plan_start_goal(self.start, self.goal)
        if res:
            for joint_conf, tool_act in joint_path:
                self.robot.reset_joint_position(joint_conf, True)
                if self.object_mover:
                    pos, ori = self.robot.get_endeffector_pose()
                    self.object_mover.match_moving_objects(pos, ori)
                time.sleep(0.005)
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
        self.update_status()
