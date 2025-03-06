import tkinter as tk
import os
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np
import sys

JOINT_INCREMENT: float = 0.1


class PathPlannerGUI:
    """
    Compact GUI for the Path Planner. Provides controls for robot joints,
    obstacle parameters, planner selections (setup, type, optimization objective,
    constraint functions), planning time, and path execution.
    """

    def __init__(self, root: tk.Tk, planner_setup, obstacle, planner_list, objective_list, constraint_list) -> None:
        self.root: tk.Tk = root

        # Support multiple planner setups.
        if isinstance(planner_setup, list):
            self.planner_setups = planner_setup
            self.planner_mappings = {p.name: p for p in self.planner_setups}
            self.selected_planner_var = tk.StringVar(
                value=list(self.planner_mappings.keys())[0])
            self.planner_setup = self.planner_mappings[self.selected_planner_var.get(
            )]
        else:
            self.planner_setups = [planner_setup]
            self.planner_mappings = {planner_setup.name: planner_setup}
            self.selected_planner_var = tk.StringVar(value=planner_setup.name)
            self.planner_setup = planner_setup

        # Planner types mapping.
        self.planner_type_mapping = {
            ptype.__name__: ptype for ptype in planner_list}
        # Use the first element from the planner_list as default.
        default_ptype = list(self.planner_type_mapping.keys())[0]
        self.selected_planner_type_var = tk.StringVar(value=default_ptype)
        self.planner_setup.setPlanner(
            self.planner_type_mapping[self.selected_planner_type_var.get()])

        # Optimization objective mapping.
        self.objective_mapping = {}
        for obj in objective_list:
            key = "None" if obj is None else (
                obj.__name__ if hasattr(obj, "__name__") else str(obj))
            self.objective_mapping[key] = obj
        # Set default to the first element in objective_list.
        default_obj_key = "None" if objective_list[0] is None else (
            objective_list[0].__name__ if hasattr(objective_list[0], "__name__") else str(objective_list[0]))
        self.selected_objective_var = tk.StringVar(value=default_obj_key)
        self.planner_setup.setOptimizationObjective(
            self.objective_mapping[self.selected_objective_var.get()])

        # Constraint functions mapping.
        self.constraint_mapping = {}
        for cf in constraint_list:
            key = "None" if cf is None else (
                cf.__name__ if hasattr(cf, "__name__") else str(cf))
            self.constraint_mapping[key] = cf
        # Set default to the first element in constraint_list.
        default_constraint_key = "None" if constraint_list[0] is None else (
            constraint_list[0].__name__ if hasattr(constraint_list[0], "__name__") else str(constraint_list[0]))
        self.selected_constraint_var = tk.StringVar(
            value=default_constraint_key)
        self.planner_setup.update_constraints(
            self.constraint_mapping[self.selected_constraint_var.get()])

        # Planning time slider variable.
        self.planning_time_var = tk.IntVar(value=5)

        # Retrieve robot and related objects.
        self.robot = self.planner_setup.robot
        self.collision_check = self.planner_setup.validity_checker.collision_check_function
        self.constraint_function = self.planner_setup.validity_checker.constraint_function
        self.object_mover = self.planner_setup.space_information.object_mover
        self.obstacle = obstacle

        self.start = self.robot.get_joint_state()
        self.goal = self.robot.get_joint_state()
        self.joint_order = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")

        self.joint_values = [tk.StringVar(value="0") for _ in self.joint_order]
        self.obstacle_values = [tk.StringVar(value="0") for _ in range(6)]
        self.collision_status = tk.StringVar(value="green")
        self.constraint_status = tk.StringVar(value="green")
        self.current_box_size = [0.5, 0.5, 0.05]

        self.create_widgets()
        self.update_joint_positions()
        self.set_initial_obstacle_values()
        self.update_status()

    def create_widgets(self) -> None:
        # Top frame: Joint and obstacle controls.
        top_frame = tk.Frame(self.root)
        top_frame.grid(row=0, column=0, padx=3, pady=3, sticky="nsew")

        joints_frame = tk.LabelFrame(top_frame, text="Joints", padx=3, pady=3)
        joints_frame.grid(row=0, column=0, padx=3, pady=3, sticky="nsew")
        for i, joint_name in enumerate(self.joint_order):
            tk.Label(joints_frame, text=joint_name).grid(
                row=i, column=0, sticky="w", padx=2, pady=1)
            tk.Button(joints_frame, text="+", command=lambda i=i: self.increment_joint(i),
                      width=2).grid(row=i, column=1, padx=2, pady=1)
            tk.Button(joints_frame, text="-", command=lambda i=i: self.decrement_joint(i),
                      width=2).grid(row=i, column=2, padx=2, pady=1)
            tk.Entry(joints_frame, textvariable=self.joint_values[i], width=6).grid(
                row=i, column=3, padx=2, pady=1)

        obstacle_frame = tk.LabelFrame(
            top_frame, text="Obstacle", padx=3, pady=3)
        obstacle_frame.grid(row=0, column=1, padx=3, pady=3, sticky="nsew")
        obs_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obs_labels):
            tk.Label(obstacle_frame, text=label).grid(
                row=i, column=0, sticky="w", padx=2, pady=1)
            tk.Button(obstacle_frame, text="+", command=lambda i=i: self.increment_obstacle(
                i), width=2).grid(row=i, column=1, padx=2, pady=1)
            tk.Button(obstacle_frame, text="-", command=lambda i=i: self.decrement_obstacle(
                i), width=2).grid(row=i, column=2, padx=2, pady=1)
            tk.Entry(obstacle_frame, textvariable=self.obstacle_values[i], width=6).grid(
                row=i, column=3, padx=2, pady=1)

        # Middle frame: Planner and status controls.
        mid_frame = tk.Frame(self.root)
        mid_frame.grid(row=1, column=0, padx=3, pady=3, sticky="ew")
        # Planner selections.
        planner_frame = tk.Frame(mid_frame)
        planner_frame.grid(row=0, column=0, padx=3, pady=3, sticky="w")
        tk.Label(planner_frame, text="Setup:").grid(
            row=0, column=0, sticky="w")
        tk.OptionMenu(planner_frame, self.selected_planner_var, *self.planner_mappings.keys(),
                      command=self.update_selected_planner).grid(row=0, column=1, padx=2)
        tk.Label(planner_frame, text="Type:").grid(row=0, column=2, sticky="w")
        tk.OptionMenu(planner_frame, self.selected_planner_type_var, *self.planner_type_mapping.keys(),
                      command=self.update_planner_type).grid(row=0, column=3, padx=2)
        tk.Label(planner_frame, text="Objective:").grid(
            row=0, column=4, sticky="w")
        tk.OptionMenu(planner_frame, self.selected_objective_var, *self.objective_mapping.keys(),
                      command=self.update_objective).grid(row=0, column=5, padx=2)
        tk.Label(planner_frame, text="Constraints:").grid(
            row=0, column=6, sticky="w")
        tk.OptionMenu(planner_frame, self.selected_constraint_var, *self.constraint_mapping.keys(),
                      command=self.update_constraints).grid(row=0, column=7, padx=2)

        # Status and planning time.
        status_frame = tk.Frame(mid_frame)
        status_frame.grid(row=0, column=1, padx=3, pady=3, sticky="e")
        tk.Label(status_frame, text="Time (s):").grid(
            row=0, column=0, sticky="w")
        tk.Scale(status_frame, variable=self.planning_time_var, from_=1, to=180,
                 orient=tk.HORIZONTAL, resolution=1, length=120).grid(row=0, column=1)
        tk.Label(status_frame, text="Collision:").grid(
            row=1, column=0, sticky="w")
        self.collision_light = tk.Label(
            status_frame, bg=self.collision_status.get(), width=6)
        self.collision_light.grid(row=1, column=1, padx=2)
        tk.Label(status_frame, text="Constraint:").grid(
            row=2, column=0, sticky="w")
        self.constraint_light = tk.Label(
            status_frame, bg=self.constraint_status.get(), width=6)
        self.constraint_light.grid(row=2, column=1, padx=2)
        # New Clearance display.
        tk.Label(status_frame, text="Clearance:").grid(
            row=3, column=0, sticky="w")
        self.clearance_label = tk.Label(status_frame, text="0.00", width=6)
        self.clearance_label.grid(row=3, column=1, padx=2)

        # Bottom frame: Path control buttons.
        bottom_frame = tk.Frame(self.root)
        bottom_frame.grid(row=2, column=0, padx=3, pady=3, sticky="ew")
        tk.Button(bottom_frame, text="Start", command=self.set_as_start,
                  width=8).grid(row=0, column=0, padx=3, pady=3)
        tk.Button(bottom_frame, text="Goal", command=self.set_as_goal,
                  width=8).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(bottom_frame, text="Plan/Execute", command=self.plan_and_execute,
                  width=12).grid(row=0, column=2, padx=3, pady=3)
        tk.Button(bottom_frame, text="Simulate G-Code", command=self.simulate_g_code,
                  width=14).grid(row=0, column=3, padx=3, pady=3)
        tk.Button(bottom_frame, text="Shrink Obs", command=self.shrink_obstacle,
                  width=10).grid(row=0, column=4, padx=3, pady=3)
        tk.Button(bottom_frame, text="Grow Obs", command=self.grow_obstacle,
                  width=10).grid(row=0, column=5, padx=3, pady=3)
        tk.Button(bottom_frame, text="Exit", command=self.root.quit,
                  width=8).grid(row=0, column=6, padx=3, pady=3)

    def update_selected_planner(self, selection: str) -> None:
        self.planner_setup = self.planner_mappings[selection]
        self.robot = self.planner_setup.robot
        self.collision_check = self.planner_setup.validity_checker.collision_check_function
        self.constraint_function = self.planner_setup.validity_checker.constraint_function or []
        self.object_mover = self.planner_setup.space_information.object_mover
        print(f"Selected setup: {selection}")

    def update_planner_type(self, selection: str) -> None:
        planner_type = self.planner_type_mapping[selection]
        self.planner_setup.setPlanner(planner_type)
        print(f"Set type: {selection}")

    def update_objective(self, selection: str) -> None:
        selected_obj = self.objective_mapping[selection]
        self.planner_setup.setOptimizationObjective(selected_obj)
        print(f"Set objective: {selection}")

    def update_constraints(self, selection: str) -> None:
        selected_constraint = self.constraint_mapping[selection]
        self.planner_setup.update_constraints(selected_constraint)
        # Update collision check functions as well.
        self.collision_check = self.planner_setup.validity_checker.collision_check_function
        self.constraint_function = self.planner_setup.validity_checker.constraint_function or []
        print(f"Set constraints: {selection}")

    def update_status(self) -> None:
        if self.object_mover:
            pos, ori = self.robot.get_endeffector_pose()
            self.object_mover.match_moving_objects(pos, ori)
        valid_collision = self.collision_check()
        self.collision_status.set("green" if valid_collision else "red")
        self.collision_light.config(bg=self.collision_status.get())
        self.update_constraint_status()
        self.update_clearance_status()

    def update_clearance_status(self) -> None:
        if self.planner_setup.validity_checker.clearance_function:
            clearance_val = self.planner_setup.validity_checker.clearance_function()
            self.clearance_label.config(text=f"{clearance_val:.2f}")
        else:
            clearance_val = "N/A"
            self.clearance_label.config(text=clearance_val)

    def update_constraint_status(self) -> None:
        valid_constraints = self.constraint_function() if self.constraint_function else True
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def shrink_obstacle(self) -> None:
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [e - e * 0.2 for e in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn)
        self.set_initial_obstacle_values()
        self.update_status()

    def grow_obstacle(self) -> None:
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [e + e * 0.2 for e in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn)
        self.set_initial_obstacle_values()
        self.update_status()

    def set_initial_obstacle_values(self) -> None:
        pos, orn_q = p.getBasePositionAndOrientation(self.obstacle)
        orn_e = p.getEulerFromQuaternion(orn_q)
        for i in range(3):
            self.obstacle_values[i].set(f"{pos[i]:.2f}")
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orn_e[i-3]:.2f}")

    def update_joint_positions(self) -> None:
        joint_state = self.robot.get_joint_state()
        for i, jn in enumerate(self.joint_order):
            self.joint_values[i].set(f"{joint_state[jn]['position']:.2f}")
        self.update_status()

    def increment_joint(self, index: int) -> None:
        cur = float(self.joint_values[index].get())
        new_val = cur + JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def decrement_joint(self, index: int) -> None:
        cur = float(self.joint_values[index].get())
        new_val = cur - JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def increment_obstacle(self, index: int) -> None:
        cur = float(self.obstacle_values[index].get())
        new_val = cur + 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index: int) -> None:
        cur = float(self.obstacle_values[index].get())
        new_val = cur - 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def update_obstacle(self) -> None:
        pos = [float(self.obstacle_values[i].get()) for i in range(3)]
        orn_e = [float(self.obstacle_values[i].get()) for i in range(3, 6)]
        orn_q = p.getQuaternionFromEuler(orn_e)
        p.resetBasePositionAndOrientation(self.obstacle, pos, orn_q)
        self.update_status()

    def apply_joint_position(self, index: int, position: float) -> None:
        joint_name = self.joint_order[index]
        self.robot.reset_joint_position({joint_name: position})
        self.update_status()

    def set_as_start(self) -> None:
        self.start = {jn: float(self.joint_values[i].get(
        )) for i, jn in enumerate(self.joint_order)}
        print("Start set:", self.start)

    def set_as_goal(self) -> None:
        self.goal = {jn: float(self.joint_values[i].get(
        )) for i, jn in enumerate(self.joint_order)}
        print("Goal set:", self.goal)

    def plan_and_execute(self) -> None:
        planning_time = self.planning_time_var.get()
        print(f"Planning (allowed time = {planning_time}s)...")
        res, joint_path = self.planner_setup.plan_start_goal(
            self.start, self.goal, allowed_time=planning_time)
        if res:
            self.g_code = pi.GCodeProcessor.joint_path_to_g_code(joint_path)
            for joint_conf, _ in joint_path:
                self.robot.reset_joint_position(joint_conf, True)
                if self.object_mover:
                    pos, ori = self.robot.get_endeffector_pose()
                    self.object_mover.match_moving_objects(pos, ori)
                time.sleep(0.005)
            self.robot.reset_joint_position(self.start, True)
            self.update_joint_positions()
            print("Execution completed.")
        else:
            print("No solution found.")
        self.update_status()

    def simulate_g_code(self) -> None:
        """
        Plans a path from start to goal, converts the resulting joint path into
        G-code using the joint_path_to_g_code function of the GCodeProcessor, and
        then iterates over the commands executing each elementary operation.
        """
        working_dir = os.path.dirname(__file__)
        g_code_path = os.path.join(working_dir, 'g_codes',
                                   'joint_path_planner.txt')
        pi.GCodeLogger.write_g_code(self.g_code, g_code_path)
        # Override the g_code attribute with the generated list.
        gcode_processor = pi.GCodeProcessor(robot=self.robot)
        gcode_processor.g_code = self.g_code

        print("Simulating G-code:")
        # Create an iterator from the processor.
        gcode_iter = iter(gcode_processor)
        for _ in gcode_iter:
            for _ in range(200):
                p.stepSimulation()
        self.update_status()
