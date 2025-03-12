import os
import sys
import time
import json
import tkinter as tk
import numpy as np
import pybullet as p
import pybullet_industrial as pi

# Constants for joint and workspace increments
JOINT_INCREMENT: float = 0.1
WORKSPACE_INCREMENT: float = 0.1  # Increment for workspace values


class ConsoleRedirector:
    """
    Redirects console output to a Tkinter Text widget.

    Args:
        text_widget (tk.Text): The Text widget to which output is redirected.
        original_stdout: The original stdout stream.
    """

    def __init__(self, text_widget: tk.Text, original_stdout) -> None:
        self.text_widget = text_widget
        self.original_stdout = original_stdout

    def write(self, s: str) -> None:
        """
        Writes string 's' to the text widget and the original stdout.

        Args:
            s (str): The string to write.
        """
        self.text_widget.insert(tk.END, s)
        self.text_widget.see(tk.END)
        self.original_stdout.write(s)

    def flush(self) -> None:
        """
        Flushes the original stdout stream.
        """
        self.original_stdout.flush()


class PathPlannerGUI:
    """
    Compact GUI for the Path Planner. Provides controls for robot joints,
    workspace (end-effector) pose, obstacle parameters, planner selections,
    and saving/loading the current GUI state.

    Args:
        root (tk.Tk): The root Tkinter window.
        planner_setup: The planner setup object or list of setups.
        obstacles: List of obstacles in the environment.
        planner_list: List of available planner types.
        objective_list: List of optimization objectives.
        constraint_list: List of constraint functions.
        working_dir (str, optional): Directory for saving GUI states.
            Defaults to os.getcwd().
    """

    def __init__(self, root: tk.Tk, planner_setup, obstacles,
                 planner_list, objective_list, constraint_list,
                 working_dir: str = os.getcwd()) -> None:
        self.root: tk.Tk = root

        # Redirect console output to GUI widget.
        self._setup_console()

        # Configure grid layout.
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=0)

        # Initialize planner setup and mappings.
        self._init_planner_setup(planner_setup, planner_list, objective_list,
                                 constraint_list)

        # Set planning time variable.
        self.planning_time_var = tk.IntVar(value=5)

        # Setup robot and obstacles.
        self.robot = self.planner_setup.robot
        self.collision_check = (
            self.planner_setup.validity_checker.collision_check_function
        )
        self.constraint_function = (
            self.planner_setup.validity_checker.constraint_function or []
        )
        self.object_mover = self.planner_setup.space_information.object_mover
        self.obstacles = obstacles
        self._init_obstacles()

        # Initialize workspace and joint control values.
        self.workspace_values = [
            tk.StringVar(value="0") for _ in range(6)
        ]
        self.start = None
        self.goal = None
        self.joint_order = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")
        self.joint_path = None
        self.joint_values = [
            tk.StringVar(value="0") for _ in self.joint_order
        ]
        self.obstacle_values = [
            tk.StringVar(value="0") for _ in range(6)
        ]
        self.collision_status = tk.StringVar(value="green")
        self.constraint_status = tk.StringVar(value="green")
        self.current_box_size = [0.5, 0.5, 0.05]

        # Initialize GUI state folder and variables.
        self.gui_states_folder = os.path.join(working_dir, 'gui_states')
        os.makedirs(self.gui_states_folder, exist_ok=True)
        self.saved_state_var = tk.StringVar(value="")
        self.gui_state_options = []

        # Create and setup widgets.
        self.create_widgets()
        self.refresh_state_dropdown()  # Load saved states.
        self.set_initial_obstacle_values()
        self.update_workspace_values()
        self.update_joint_positions()  # Initialize joint fields.
        self.update_status()

    # ------------------- Initialization Helper Methods -------------------
    def _setup_console(self) -> None:
        """
        Sets up the console output redirection to the text widget.
        """
        console_frame = tk.Frame(self.root)
        console_frame.grid(row=0, column=1, padx=3, pady=3, sticky="nsew")
        tk.Label(console_frame, text="Console Output").pack(
            side=tk.TOP, anchor="w"
        )
        self.console_text = tk.Text(console_frame, height=14, width=40)
        self.console_text.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        original_stdout = sys.stdout
        sys.stdout = ConsoleRedirector(self.console_text, original_stdout)

    def _init_planner_setup(self, planner_setup, planner_list, objective_list,
                            constraint_list) -> None:
        """
        Initializes the planner setup and associated mappings.

        Args:
            planner_setup: Planner setup object or list of setups.
            planner_list: List of available planner types.
            objective_list: List of optimization objectives.
            constraint_list: List of constraint functions.
        """
        if isinstance(planner_setup, list):
            self.planner_setups = planner_setup
            self.planner_mappings = {
                p_obj.name: p_obj for p_obj in self.planner_setups
            }
            first_name = list(self.planner_mappings.keys())[0]
            self.selected_planner_var = tk.StringVar(value=first_name)
            self.planner_setup = self.planner_mappings[first_name]
        else:
            self.planner_setups = [planner_setup]
            self.planner_mappings = {planner_setup.name: planner_setup}
            self.selected_planner_var = tk.StringVar(
                value=planner_setup.name
            )
            self.planner_setup = planner_setup

        self.planner_type_mapping = {
            ptype.__name__: ptype for ptype in planner_list
        }
        default_ptype = list(self.planner_type_mapping.keys())[0]
        self.selected_planner_type_var = tk.StringVar(value=default_ptype)
        self.planner_setup.setPlanner(
            self.planner_type_mapping[self.selected_planner_type_var.get()]
        )

        # Initialize optimization objective mapping.
        self.objective_mapping = {}
        for obj in objective_list:
            key = "None" if obj is None else (
                obj.__name__ if hasattr(obj, "__name__") else str(obj)
            )
            self.objective_mapping[key] = obj
        default_obj_key = (
            "None" if objective_list[0] is None
            else (objective_list[0].__name__
                  if hasattr(objective_list[0], "__name__")
                  else str(objective_list[0]))
        )
        self.selected_objective_var = tk.StringVar(value=default_obj_key)
        self.planner_setup.setOptimizationObjective(
            self.objective_mapping[self.selected_objective_var.get()]
        )

        # Initialize constraint mapping.
        self.constraint_mapping = {}
        for cf in constraint_list:
            key = "None" if cf is None else (
                cf.__name__ if hasattr(cf, "__name__") else str(cf)
            )
            self.constraint_mapping[key] = cf
        default_constraint_key = (
            "None" if constraint_list[0] is None
            else (constraint_list[0].__name__
                  if hasattr(constraint_list[0], "__name__")
                  else str(constraint_list[0]))
        )
        self.selected_constraint_var = tk.StringVar(
            value=default_constraint_key
        )
        self.planner_setup.update_constraints(
            self.constraint_mapping[self.selected_constraint_var.get()]
        )

    def _init_obstacles(self) -> None:
        """
        Initializes obstacle names and the selected obstacle variable.
        """
        self.obstacle_names = []
        for obs in self.obstacles:
            info = p.getBodyInfo(obs)
            name = (info[1].decode("utf-8")
                    if isinstance(info[1], bytes)
                    else info[1])
            self.obstacle_names.append(name)
        default_obs = self.obstacle_names[0] if self.obstacle_names else "None"
        self.selected_obstacle_str = tk.StringVar(value=default_obs)

    # ------------------- Widget Creation Methods -------------------
    def create_widgets(self) -> None:
        """
        Creates all widgets for the GUI.
        """
        self._create_top_left_frame()
        self._create_mid_frame()
        self._create_bottom_frame()
        self._create_state_frame()

    def _create_top_left_frame(self) -> None:
        """
        Creates the top left frame with joint, workspace, and obstacle
        controls.
        """
        top_left_frame = tk.Frame(self.root)
        top_left_frame.grid(row=0, column=0, padx=3, pady=3, sticky="nsew")
        self._create_joints_frame(top_left_frame)
        self._create_workspace_frame(top_left_frame)
        self._create_obstacle_frame(top_left_frame)

    def _create_joints_frame(self, parent: tk.Frame) -> None:
        """
        Creates the joint space control panel with an Update button to set the
        robot's joint state from the GUI values.

        Args:
            parent (tk.Frame): Parent frame to contain the widget.
        """
        joints_frame = tk.LabelFrame(parent, text="Joint Space",
                                     padx=3, pady=3)
        joints_frame.grid(row=0, column=0, padx=3, pady=3, sticky="nsew")
        # Update button to apply manual joint value changes.
        tk.Button(
            joints_frame,
            text="Update",
            command=self.set_joint_position,
            width=8
        ).grid(row=0, column=0, columnspan=4, padx=2, pady=2)
        for i, joint_name in enumerate(self.joint_order):
            row = i + 1  # Shift rows down by one for the update button.
            tk.Label(joints_frame, text=joint_name).grid(
                row=row, column=0, sticky="w", padx=2, pady=1
            )
            tk.Button(
                joints_frame,
                text="+",
                command=lambda i=i: self.increment_joint(i),
                width=2
            ).grid(row=row, column=1, padx=2, pady=1)
            tk.Button(
                joints_frame,
                text="-",
                command=lambda i=i: self.decrement_joint(i),
                width=2
            ).grid(row=row, column=2, padx=2, pady=1)
            tk.Entry(
                joints_frame,
                textvariable=self.joint_values[i],
                width=6
            ).grid(row=row, column=3, padx=2, pady=1)

    def _create_workspace_frame(self, parent: tk.Frame) -> None:
        """
        Creates the workspace control panel with an Update button to set the
        robot's end-effector pose from the GUI values.

        Args:
            parent (tk.Frame): Parent frame to contain the widget.
        """
        workspace_frame = tk.LabelFrame(parent, text="Workspace Control",
                                        padx=3, pady=3)
        workspace_frame.grid(row=0, column=1, padx=3, pady=3, sticky="nsew")
        # Update button to apply manual workspace pose changes.
        tk.Button(
            workspace_frame,
            text="Update",
            command=self.set_workspace_pose,
            width=8
        ).grid(row=0, column=0, columnspan=4, padx=2, pady=2)
        ws_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(ws_labels):
            row = i + 1  # Shift rows down by one for the update button.
            tk.Label(workspace_frame, text=label).grid(
                row=row, column=0, sticky="w", padx=2, pady=1
            )
            tk.Button(
                workspace_frame,
                text="-",
                command=lambda i=i: self.decrement_workspace(i),
                width=2
            ).grid(row=row, column=1, padx=2, pady=1)
            tk.Entry(
                workspace_frame,
                textvariable=self.workspace_values[i],
                width=6
            ).grid(row=row, column=3, padx=2, pady=1)
            tk.Button(
                workspace_frame,
                text="+",
                command=lambda i=i: self.increment_workspace(i),
                width=2
            ).grid(row=row, column=2, padx=2, pady=1)

    def _create_obstacle_frame(self, parent: tk.Frame) -> None:
        """
        Creates the obstacle control panel.

        Args:
            parent (tk.Frame): Parent frame to contain the widget.
        """
        obstacle_frame = tk.LabelFrame(parent, text="Obstacle",
                                       padx=3, pady=3)
        obstacle_frame.grid(row=0, column=2, padx=3, pady=3, sticky="nsew")
        tk.Label(obstacle_frame, text="Select").grid(
            row=0, column=0, sticky="w", padx=2, pady=1
        )
        tk.OptionMenu(
            obstacle_frame,
            self.selected_obstacle_str,
            *self.obstacle_names,
            command=lambda _: self.set_initial_obstacle_values()
        ).grid(row=0, column=1, padx=2, pady=1)
        obs_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obs_labels):
            tk.Label(obstacle_frame, text=label).grid(
                row=i+1, column=0, sticky="w", padx=2, pady=1
            )
            tk.Button(
                obstacle_frame,
                text="+",
                command=lambda i=i: self.increment_obstacle(i),
                width=2
            ).grid(row=i+1, column=1, padx=2, pady=1)
            tk.Button(
                obstacle_frame,
                text="-",
                command=lambda i=i: self.decrement_obstacle(i),
                width=2
            ).grid(row=i+1, column=2, padx=2, pady=1)
            tk.Entry(
                obstacle_frame,
                textvariable=self.obstacle_values[i],
                width=6
            ).grid(row=i+1, column=3, padx=2, pady=1)

    def _create_mid_frame(self) -> None:
        """
        Creates the mid frame with planner and status controls.
        """
        mid_frame = tk.Frame(self.root)
        mid_frame.grid(row=1, column=0, columnspan=2,
                       padx=3, pady=3, sticky="ew")
        planner_frame = tk.Frame(mid_frame)
        planner_frame.grid(row=0, column=0, padx=3, pady=3, sticky="w")
        tk.Label(planner_frame, text="Setup:").grid(
            row=0, column=0, sticky="w"
        )
        tk.OptionMenu(
            planner_frame,
            self.selected_planner_var,
            *self.planner_mappings.keys(),
            command=self.update_selected_planner
        ).grid(row=0, column=1, padx=2)
        tk.Label(planner_frame, text="Type:").grid(
            row=0, column=2, sticky="w"
        )
        tk.OptionMenu(
            planner_frame,
            self.selected_planner_type_var,
            *self.planner_type_mapping.keys(),
            command=self.update_planner_type
        ).grid(row=0, column=3, padx=2)
        tk.Label(planner_frame, text="Objective:").grid(
            row=0, column=4, sticky="w"
        )
        tk.OptionMenu(
            planner_frame,
            self.selected_objective_var,
            *self.objective_mapping.keys(),
            command=self.update_objective
        ).grid(row=0, column=5, padx=2)
        tk.Label(planner_frame, text="Constraints:").grid(
            row=0, column=6, sticky="w"
        )
        tk.OptionMenu(
            planner_frame,
            self.selected_constraint_var,
            *self.constraint_mapping.keys(),
            command=self.update_constraints
        ).grid(row=0, column=7, padx=2)

        status_frame = tk.Frame(mid_frame)
        status_frame.grid(row=0, column=1, padx=3, pady=3, sticky="e")
        tk.Label(status_frame, text="Time (s):").grid(
            row=0, column=0, sticky="w"
        )
        tk.Scale(
            status_frame,
            variable=self.planning_time_var,
            from_=1,
            to=180,
            orient=tk.HORIZONTAL,
            resolution=1,
            length=120
        ).grid(row=0, column=1)
        tk.Label(status_frame, text="Collision:").grid(
            row=1, column=0, sticky="w"
        )
        self.collision_light = tk.Label(
            status_frame,
            bg=self.collision_status.get(),
            width=6
        )
        self.collision_light.grid(row=1, column=1, padx=2)
        tk.Label(status_frame, text="Constraint:").grid(
            row=2, column=0, sticky="w"
        )
        self.constraint_light = tk.Label(
            status_frame,
            bg=self.constraint_status.get(),
            width=6
        )
        self.constraint_light.grid(row=2, column=1, padx=2)
        tk.Label(status_frame, text="Clearance:").grid(
            row=3, column=0, sticky="w"
        )
        self.clearance_label = tk.Label(status_frame, text="0.00", width=6)
        self.clearance_label.grid(row=3, column=1, padx=2)

    def _create_bottom_frame(self) -> None:
        """
        Creates the bottom frame with path control buttons.
        """
        bottom_frame = tk.Frame(self.root)
        bottom_frame.grid(row=2, column=0, columnspan=2,
                          padx=3, pady=3, sticky="ew")
        tk.Button(
            bottom_frame, text="Start", command=self.set_as_start, width=8
        ).grid(row=0, column=0, padx=3, pady=3)
        tk.Button(
            bottom_frame, text="Goal", command=self.set_as_goal, width=8
        ).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(
            bottom_frame, text="Plan", command=self.plan, width=8
        ).grid(row=0, column=2, padx=3, pady=3)
        tk.Button(
            bottom_frame, text="Run", command=self.run, width=8
        ).grid(row=0, column=3, padx=3, pady=3)
        tk.Button(
            bottom_frame, text="Exit", command=self.root.quit, width=8
        ).grid(row=0, column=4, padx=3, pady=3)

    def _create_state_frame(self) -> None:
        """
        Creates the GUI state controls frame.
        """
        state_frame = tk.Frame(self.root)
        state_frame.grid(row=3, column=0, columnspan=2,
                         padx=3, pady=3, sticky="ew")
        tk.Label(state_frame, text="Saved GUI States:").grid(
            row=0, column=0, sticky="w", padx=2
        )
        self.state_dropdown = tk.OptionMenu(
            state_frame, self.saved_state_var, ()
        )
        self.state_dropdown.grid(row=0, column=1, padx=2, pady=2)
        tk.Button(
            state_frame,
            text="Save State",
            command=self.save_gui_state,
            width=10
        ).grid(row=0, column=2, padx=2, pady=2)

    # ------------------- State Save/Load Methods -------------------
    def refresh_state_dropdown(self) -> None:
        """
        Refreshes the dropdown menu with all JSON state files.

        Returns:
            None
        """
        files = [f for f in os.listdir(self.gui_states_folder)
                 if f.endswith('.json')]
        files.sort()
        self.gui_state_options = files
        menu = self.state_dropdown["menu"]
        menu.delete(0, "end")
        for file in files:
            menu.add_command(
                label=file,
                command=lambda file=file: self.load_gui_state(file)
            )
        self.saved_state_var.set(files[0] if files else "")

    def save_gui_state(self) -> None:
        """
        Saves the current GUI state to a JSON file.

        Returns:
            None
        """
        joint_path_values = []
        if self.joint_path is not None:
            joint_path_values = self.joint_path.joint_values.tolist()

        # Save each obstacle's position and orientation (Euler angles).
        obstacles_state = {}
        for obs in self.obstacles:
            pos, orn_q = p.getBasePositionAndOrientation(obs)
            orn_e = p.getEulerFromQuaternion(orn_q)
            info = p.getBodyInfo(obs)
            name = (info[1].decode("utf-8")
                    if isinstance(info[1], bytes)
                    else info[1])
            obstacles_state[name] = {"pos": pos, "orn_e": orn_e}

        robot_information = []
        robot_urdf = []
        for index, p_setup in enumerate(self.planner_setups):
            if p_setup.robot.urdf not in robot_urdf:
                robot_urdf.append(p_setup.robot.urdf)
                joint_state = p_setup.robot.get_joint_position()
                robot_information.append((index, joint_state))

        state = {
            "selected_obstacle": self.selected_obstacle_str.get(),
            "planner_setup": self.selected_planner_var.get(),
            "planner_type": self.selected_planner_type_var.get(),
            "objective": self.selected_objective_var.get(),
            "constraint": self.selected_constraint_var.get(),
            "planning_time": self.planning_time_var.get(),
            "joint_path_values": joint_path_values,
            "start": self.start,
            "goal": self.goal,
            "obstacles_state": obstacles_state,
            "robot_information": robot_information
        }
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"state_{timestamp}.json"
        full_path = os.path.join(self.gui_states_folder, filename)
        with open(full_path, "w") as f:
            json.dump(state, f, indent=4)
        print(f"Saved GUI state to {filename}")
        self.refresh_state_dropdown()

    def load_gui_state(self, filename: str) -> None:
        """
        Loads a saved GUI state from a JSON file.

        Args:
            filename (str): Name of the JSON state file.

        Returns:
            None
        """
        full_path = os.path.join(self.gui_states_folder, filename)
        with open(full_path, "r") as f:
            state = json.load(f)

        # Load robot information.
        for index, joint_state in state["robot_information"]:
            self.planner_setups[index].robot.reset_joint_position(joint_state)

        self.selected_obstacle_str.set(state["selected_obstacle"])
        self.selected_planner_var.set(state["planner_setup"])
        self.selected_planner_type_var.set(state["planner_type"])
        self.selected_objective_var.set(state["objective"])
        self.selected_constraint_var.set(state["constraint"])
        self.planning_time_var.set(state["planning_time"])

        # Reload joint path, start, and goal configurations.
        if state["joint_path_values"]:
            self.joint_path = pi.JointPath(
                np.array(state["joint_path_values"]), self.joint_order
            )
            self.start = state["start"]
            self.goal = state["goal"]
        print(f"Loaded GUI state from {filename}")

        # Reset obstacles' positions and orientations.
        if "obstacles_state" in state:
            obstacles_state = state["obstacles_state"]
            for obs, name in zip(self.obstacles, self.obstacle_names):
                if name in obstacles_state:
                    obs_state = obstacles_state[name]
                    pos = obs_state["pos"]
                    orn_e = obs_state["orn_e"]
                    orn_q = p.getQuaternionFromEuler(orn_e)
                    p.resetBasePositionAndOrientation(obs, pos, orn_q)

        self.update_joint_positions()
        self.update_workspace_values()
        self.update_constraints(state["constraint"])
        self.update_selected_planner(state["planner_setup"])
        self.update_planner_type(state["planner_type"])
        self.update_objective(state["objective"])
        self.set_initial_obstacle_values()
        self.get_current_obstacle()
        self.update_obstacle()
        self.update_status()

    # ------------------- Update Methods -------------------
    def update_selected_planner(self, selection: str) -> None:
        """
        Updates the selected planner setup and associated variables.

        Args:
            selection (str): The name of the planner setup selected.

        Returns:
            None
        """
        self.planner_setup = self.planner_mappings[selection]
        self.robot = self.planner_setup.robot
        self.collision_check = (
            self.planner_setup.validity_checker.collision_check_function
        )
        self.constraint_function = (
            self.planner_setup.validity_checker.constraint_function or []
        )
        self.object_mover = self.planner_setup.space_information.object_mover
        self.update_workspace_values()
        self.update_joint_positions()
        print(f"Selected setup: {selection}")

    def update_planner_type(self, selection: str) -> None:
        """
        Updates the planner type based on user selection.

        Args:
            selection (str): The selected planner type name.

        Returns:
            None
        """
        planner_type = self.planner_type_mapping[selection]
        self.planner_setup.setPlanner(planner_type)
        print(f"Set type: {selection}")

    def update_objective(self, selection: str) -> None:
        """
        Updates the optimization objective based on user selection.

        Args:
            selection (str): The name of the selected objective.

        Returns:
            None
        """
        selected_obj = self.objective_mapping[selection]
        self.planner_setup.setOptimizationObjective(selected_obj)
        print(f"Set objective: {selection}")

    def update_constraints(self, selection: str) -> None:
        """
        Updates the constraint functions based on user selection.

        Args:
            selection (str): The name of the selected constraint.

        Returns:
            None
        """
        selected_constraint = self.constraint_mapping[selection]
        self.planner_setup.update_constraints(selected_constraint)
        self.collision_check = (
            self.planner_setup.validity_checker.collision_check_function
        )
        self.constraint_function = (
            self.planner_setup.validity_checker.constraint_function or []
        )
        print(f"Set constraints: {selection}")

    def update_status(self) -> None:
        """
        Updates the status of collision, constraints, and clearance.

        Returns:
            None
        """
        if self.object_mover:
            pos, ori_q = self.robot.get_endeffector_pose()
            self.object_mover.match_moving_objects(pos, ori_q)
        valid_collision = self.collision_check()
        self.collision_status.set("green" if valid_collision else "red")
        self.collision_light.config(bg=self.collision_status.get())
        self.update_constraint_status()
        self.update_clearance_status()

    def update_clearance_status(self) -> None:
        """
        Updates the clearance status using the clearance function.

        Returns:
            None
        """
        if self.planner_setup.validity_checker.clearance_function:
            clearance_val = (
                self.planner_setup.validity_checker.clearance_function()
            )
            self.clearance_label.config(text=f"{clearance_val:.2f}")
        else:
            self.clearance_label.config(text="N/A")

    def update_constraint_status(self) -> None:
        """
        Updates the constraint status indicator.

        Returns:
            None
        """
        valid_constraints = (self.constraint_function()
                             if self.constraint_function else True)
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def update_workspace_values(self) -> None:
        """
        Updates workspace control fields from the robot's end-effector pose.

        Returns:
            None
        """
        pos, ori_q = self.robot.get_endeffector_pose()
        euler = p.getEulerFromQuaternion(ori_q)
        for i in range(3):
            self.workspace_values[i].set(f"{pos[i]:.2f}")
        for i in range(3, 6):
            self.workspace_values[i].set(f"{euler[i-3]:.2f}")

    def update_joint_positions(self) -> None:
        """
        Updates joint control fields from the current robot joint state.

        Returns:
            None
        """
        joint_state = self.robot.get_joint_state()
        for i, jn in enumerate(self.joint_order):
            self.joint_values[i].set(f"{joint_state[jn]['position']:.2f}")

    # ------------------- Workspace and Joint Methods -------------------
    def set_workspace_pose(self) -> None:
        """
        Sets the robot's end-effector pose based on workspace values.

        Returns:
            None
        """
        try:
            pos = [float(self.workspace_values[i].get())
                   for i in range(3)]
            euler = [float(self.workspace_values[i].get())
                     for i in range(3, 6)]
            quat = p.getQuaternionFromEuler(euler)
            self.robot.reset_endeffector_pose(np.array(pos),
                                              np.array(quat))
        except Exception as e:
            print("Error setting workspace pose:", e)
        self.update_workspace_values()
        self.update_joint_positions()
        self.update_status()

    def increment_workspace(self, index: int) -> None:
        """
        Increments a workspace value and updates the pose.

        Args:
            index (int): Index of the workspace parameter to increment.

        Returns:
            None
        """
        cur = float(self.workspace_values[index].get())
        new_val = cur + WORKSPACE_INCREMENT
        self.workspace_values[index].set(f"{new_val:.2f}")
        self.set_workspace_pose()

    def decrement_workspace(self, index: int) -> None:
        """
        Decrements a workspace value and updates the pose.

        Args:
            index (int): Index of the workspace parameter to decrement.

        Returns:
            None
        """
        cur = float(self.workspace_values[index].get())
        new_val = cur - WORKSPACE_INCREMENT
        self.workspace_values[index].set(f"{new_val:.2f}")
        self.set_workspace_pose()

    def increment_joint(self, index: int) -> None:
        """
        Increments a joint value if within limits.

        Args:
            index (int): Index of the joint to increment.

        Returns:
            None
        """
        cur = float(self.joint_values[index].get())
        new_val = cur + JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.set_joint_position()

    def decrement_joint(self, index: int) -> None:
        """
        Decrements a joint value if within limits.

        Args:
            index (int): Index of the joint to decrement.

        Returns:
            None
        """
        cur = float(self.joint_values[index].get())
        new_val = cur - JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.set_joint_position()

    def set_joint_position(self) -> None:
        """
        Sets the robot joint positions from the GUI input values.
        Validates joint values and clamps them to allowed limits.

        Returns:
            None
        """
        joint_positions = {}
        for i, joint_name in enumerate(self.joint_order):
            try:
                val = float(self.joint_values[i].get())
            except ValueError:
                # If invalid input, default to lower limit.
                val = self.joint_limits[0][joint_name]
            lower = self.joint_limits[0][joint_name]
            upper = self.joint_limits[1][joint_name]
            # Clamp the value to the range [lower, upper]
            if val < lower:
                val = lower
            elif val > upper:
                val = upper
            # Update the GUI entry with the clamped value.
            self.joint_values[i].set(f"{val:.2f}")
            joint_positions[joint_name] = val

        self.robot.reset_joint_position(joint_positions)
        self.update_workspace_values()
        self.update_status()


    # ------------------- Obstacle Methods -------------------
    def set_initial_obstacle_values(self) -> None:
        """
        Sets the initial values for the selected obstacle.

        Returns:
            None
        """
        if self.obstacles:
            obstacle_id, _ = self.get_current_obstacle()
            pos, orn_q = p.getBasePositionAndOrientation(obstacle_id)
            orn_e = p.getEulerFromQuaternion(orn_q)
            for i in range(3):
                self.obstacle_values[i].set(f"{pos[i]:.2f}")
            for i in range(3, 6):
                self.obstacle_values[i].set(f"{orn_e[i-3]:.2f}")
        else:
            return

    def increment_obstacle(self, index: int) -> None:
        """
        Increments an obstacle parameter and updates it.

        Args:
            index (int): Index of the obstacle parameter to increment.

        Returns:
            None
        """
        cur = float(self.obstacle_values[index].get())
        new_val = cur + 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index: int) -> None:
        """
        Decrements an obstacle parameter and updates it.

        Args:
            index (int): Index of the obstacle parameter to decrement.

        Returns:
            None
        """
        cur = float(self.obstacle_values[index].get())
        new_val = cur - 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def update_obstacle(self) -> None:
        """
        Updates the selected obstacle's position and orientation.

        Returns:
            None
        """
        if self.obstacles:
            pos = [float(self.obstacle_values[i].get())
                   for i in range(3)]
            orn_e = [float(self.obstacle_values[i].get())
                     for i in range(3, 6)]
            orn_q = p.getQuaternionFromEuler(orn_e)
            obstacle_id, idx = self.get_current_obstacle()
            p.resetBasePositionAndOrientation(obstacle_id, pos, orn_q)
            self.update_status()
        else:
            return

    def get_current_obstacle(self):
        """
        Returns the current obstacle and its index.

        Returns:
            Tuple: (obstacle object or None, index or -1)
        """
        if self.obstacles:
            selected_name = self.selected_obstacle_str.get()
            if selected_name in self.obstacle_names:
                idx = self.obstacle_names.index(selected_name)
                return self.obstacles[idx], idx
            return None, -1
        else:
            return

    # ------------------- Path Planning Methods -------------------
    def set_as_start(self) -> None:
        """
        Sets the current joint configuration as the start configuration.

        Returns:
            None
        """
        self.start = {
            jn: float(self.joint_values[i].get())
            for i, jn in enumerate(self.joint_order)
        }
        print("Start set:", self.start)

    def set_as_goal(self) -> None:
        """
        Sets the current joint configuration as the goal configuration.

        Returns:
            None
        """
        self.goal = {
            jn: float(self.joint_values[i].get())
            for i, jn in enumerate(self.joint_order)
        }
        print("Goal set:", self.goal)

    def plan(self) -> None:
        """
        Plans a path from start to goal using the selected planner.

        Returns:
            None
        """
        planning_time = self.planning_time_var.get()
        print(f"Planning (allowed time = {planning_time}s)...")
        if self.start is None or self.goal is None:
            print("Error: Start and goal configurations must be set "
                  "before planning.")
            return
        try:
            result, joint_path = self.planner_setup.plan_start_goal(
                self.start, self.goal, allowed_time=planning_time
            )
            if not result:
                print("Planning failed: No solution found within "
                      "the given time.")
                self.joint_path = None
            else:
                self.joint_path = joint_path
                print("Planning succeeded.")
        except Exception as e:
            print("Error during planning:", e)

    def run(self) -> None:
        """
        Executes the planned joint path.

        Returns:
            None
        """
        if self.joint_path:
            for joint_conf, _ in self.joint_path:
                self.robot.reset_joint_position(joint_conf)
                if self.object_mover:
                    pos, ori = self.robot.get_endeffector_pose()
                    self.object_mover.match_moving_objects(pos, ori)
                time.sleep(0.03)
            print("Execution completed.")
        else:
            print("No execution path available.")
        self.update_status()
